#!/usr/bin/env python3
#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#  (Adapted from original ROSA example)
#
#  Modifications for TurtleBot3 Navigation with move_base

# --- ROS Imports ---
import rospy
import actionlib
import tf # For transformations
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler

# --- Standard Python Imports ---
import asyncio
import os
import dotenv
import pyinputplus as pyip
import numpy as np

# --- Langchain/ROSA Imports ---
from langchain.agents import tool, Tool
from rich.console import Console
from rich.live import Live
from rich.markdown import Markdown
from rich.panel import Panel
from rich.text import Text
from rosa import ROSA

# --- Local Project Imports (Ensure these files exist and are correct) ---
try:
    # Assuming these are in the same directory or PYTHONPATH is set correctly by ROS
    from help import get_help
    from llm import get_llm
    from prompts import get_prompts
    # Use relative import for locations if in the same package/directory
    from . import locations
except ImportError:
    # Fallback for direct execution or if structure differs
    try:
        import help
        import llm
        import prompts
        import locations
        get_help = help.get_help
        get_llm = llm.get_llm
        get_prompts = prompts.get_prompts
    except ImportError as e:
        rospy.logerr(f"Failed to import local modules (help, llm, prompts, locations): {e}")
        rospy.logerr("Make sure these files exist and are accessible (e.g., __init__.py in scripts folder).")
        # Depending on severity, you might want to exit or continue without these features
        # For now, define stubs so the rest of the code doesn't break immediately
        def get_help(examples): return "Help unavailable."
        def get_llm(streaming): return None # This will likely cause issues later if LLM is needed
        def get_prompts(): return {}


class TurtleBotAgent(ROSA):
    def __init__(self, streaming: bool = True, verbose: bool = True):

        # ---- Navigation Action Client Setup ----
        # IMPORTANT: Initialize ROS node *before* this if __init__ needs ROS.
        # Node is initialized in main() before creating this instance.
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_available = False # Default to not available
        rospy.loginfo("Waiting for /move_base action server...")
        try:
            # Increase timeout if necessary, especially on simulation startup
            if not self.move_base_client.wait_for_server(rospy.Duration(10.0)): # Reduced timeout slightly
                 rospy.logerr("/move_base action server not available! Navigation tool will fail.")
            else:
                 rospy.loginfo("/move_base action server found.")
                 self.move_base_available = True
        except rospy.ROSException as e:
             rospy.logerr(f"ROSException while waiting for move_base server: {e}")
             rospy.logerr("Is roscore running? Is the navigation stack running?")


        self.navigation_active = False
        self.last_nav_result = None
        self.last_nav_status = None
        # ---- End Navigation Setup ----

        # --- Tool Definition ---
        # Define the tool function that will call the instance method
        # Needs access to 'self', so we define it here or pass 'self' if defined globally
        @tool
        def navigate_to_location_tool(location_name: str) -> str:
            """
            Navigates the TurtleBot3 robot to a predefined named location like L1, L2, CENTER, POINT_A, etc.
            Uses the /move_base action server. Input MUST be one of the known location names.
            Use the 'locations' command to see known locations.
            Returns a string indicating success ('Navigation successful.') or failure ('Navigation failed: [reason]').
            """
            # Calls the synchronous method within the agent instance
            return self.send_navigation_goal_sync(location_name)


        # --- ROSA Initialization ---
        self.__blacklist = [] # Define blacklist if needed
        self.__prompts = get_prompts() # Load prompts
        self.__llm = get_llm(streaming=streaming) # Load LLM

        # Check if LLM loaded correctly
        if self.__llm is None and get_llm != None : # Avoid error if stub was used
             rospy.logwarn("LLM failed to initialize. Agent capabilities may be limited.")
             # Handle LLM initialization failure gracefully if possible

        # Initialize the ROSA base class
        super().__init__(
            ros_version=1,
            llm=self.__llm,
            tools=[navigate_to_location_tool], # Pass the tool instance
            tool_packages=[], # Add others like ur_tools if needed elsewhere
            blacklist=self.__blacklist,
            prompts=self.__prompts,
            verbose=verbose,
            accumulate_chat_history=False, # Set as needed
            streaming=streaming,
        )

        # --- Logging registered tools (Corrected) ---
        rospy.loginfo("ROSA Agent Initialized with Tools:")
        # Access tools via self.agent.tools after super().__init__
        if hasattr(self, 'agent') and hasattr(self.agent, 'tools'):
            for t in self.agent.tools:
                rospy.loginfo(f" - Tool Name: '{t.name}'")
                # Log simplified description - full schema can be long
                rospy.loginfo(f"     Description: {t.description.splitlines()[0]}...") # Log first line
        else:
            rospy.logwarn("Could not find self.agent.tools attribute to log.")
        # ---- End Tool Logging ----

        # --- Command Handling ---
        known_locs_str = ', '.join(locations.get_known_locations()) if locations else "Unavailable"
        self.examples = [
            "Go to L1",
            "Navigate to CENTER",
            "Move the robot to POINT_A",
            f"What locations do you know?"
        ]

        self.command_handler = {
            "help": lambda: self.submit(get_help(self.examples)),
            "examples": lambda: self.submit(self.choose_example()),
            "clear": self.clear, # Use async version directly if needed elsewhere
            "locations": lambda: self.submit(f"I know these locations: {known_locs_str}"),
        }
        # --- End Command Handling ---


    # --- ROSA Interface Methods ---
    @property
    def greeting(self):
        greeting = Text(
            "\nHi! I'm the ROSA-TurtleBot3 agent 🤖. How can I help you navigate?\n"
        )
        greeting.stylize("frame bold green")
        try:
            known_locs = locations.get_known_locations()
            greeting.append(
                f"I know locations like {known_locs[0]}, {known_locs[1]}...\n",
                style="italic cyan"
            )
        except Exception: # Handle case where locations might fail to load
             greeting.append(
                f"I can navigate to predefined locations.\n",
                style="italic cyan"
            )
        greeting.append(
            f"Try 'go to L1', {', '.join(self.command_handler.keys())} or exit.",
            style="italic",
        )
        return greeting

    def choose_example(self):
        """Get user selection from the list of examples."""
        return pyip.inputMenu(
            self.examples,
            prompt="\nEnter your choice and press enter: \n",
            numbered=True,
            blank=False,
            timeout=60,
            default="1",
        )

    async def clear(self):
        """Clear the chat history."""
        self.clear_chat()
        # self.last_events = [] # If ROSA base class uses this
        os.system("clear")

    def get_input(self, prompt: str):
        """Get user input from the console."""
        try:
            return pyip.inputStr(prompt, default="help")
        except pyip.TimeoutException:
            rospy.logwarn("Input timed out.")
            return "help" # Default to help on timeout

    async def run(self):
        """ Run the agent's main interaction loop. """
        await self.clear()
        console = Console()

        while not rospy.is_shutdown(): # Check ROS shutdown status
            console.print(self.greeting)
            try:
                user_input = self.get_input("> ")
            except EOFError: # Handle Ctrl+D
                rospy.loginfo("EOF received, exiting.")
                break

            if user_input.lower() == "exit":
                break
            elif user_input.lower() in self.command_handler:
                handler_func = self.command_handler[user_input.lower()]
                # Check if the handler is async and await if necessary
                if asyncio.iscoroutinefunction(handler_func):
                    await handler_func()
                elif asyncio.iscoroutine(handler_func): # If it's already a coroutine object
                     await handler_func
                else:
                    # If it's a regular function that might call submit (which is async)
                    # We need to handle this carefully. Let's assume submit handles async correctly.
                    # Or better, make command handlers return strings and call submit here.
                     result_or_coro = handler_func()
                     if isinstance(result_or_coro, str):
                         await self.submit(result_or_coro) # Submit the string result
                     elif asyncio.iscoroutine(result_or_coro):
                         await result_or_coro # Await if it returned a coroutine

            else:
                await self.submit(user_input)

            await asyncio.sleep(0.1) # Prevent tight loop if input is processed quickly

        rospy.loginfo("ROSA agent run loop finished.")


    async def submit(self, query: str):
        """ Submit query to ROSA and print response using Rich Live display. """
        if self.__llm is None:
             rospy.logerr("LLM is not initialized. Cannot process query.")
             # Optionally display an error message to the user via Console
             return

        console = Console()
        # Use a simple print for now, Live can be complex with async/ROS interaction
        console.print(f"[bold blue]You:[/bold blue] {query}")
        console.print("[bold green]Agent:[/bold green]", end="")
        response_text = ""
        try:
            # Use astream to get chunks
            async for chunk in self.astream(query):
                # Handle different chunk types from Langchain/ROSA astream
                content_to_print = ""
                if isinstance(chunk, str):
                    content_to_print = chunk
                elif isinstance(chunk, dict):
                    # Look for common keys where content might be
                    if 'output' in chunk: content_to_print = chunk['output']
                    elif 'content' in chunk: content_to_print = chunk['content']
                    elif 'response' in chunk: content_to_print = chunk['response']

                if content_to_print:
                    response_text += content_to_print
                    console.print(content_to_print, end="") # Print chunk immediately

            console.print() # Add a newline after the full response

        except Exception as e:
            rospy.logerr(f"Error during ROSA invocation: {e}")
            console.print(f"\n[bold red]Error processing request: {e}[/bold red]")
        finally:
            pass # No live.stop() needed if not using Live

    # --- Navigation Helper Methods ---
    def send_navigation_goal_sync(self, location_name: str) -> str:
        """
        Looks up location, sends goal, AND WAITS for the result.
        Returns a status string for the LLM Tool.
        """
        rospy.loginfo(f"Tool 'navigate_to_location_tool' called with location: '{location_name}'")

        if not self.move_base_available:
            rospy.logerr("Move Base server not available. Cannot navigate.")
            return "Navigation failed: Action server not available."

        # Check connection again before sending
        if not self.move_base_client.gh:
            rospy.logerr("Move Base server connection lost.")
            if not self.move_base_client.wait_for_server(rospy.Duration(2.0)):
                 rospy.logerr("Failed to reconnect to Move Base server.")
                 return "Navigation failed: Action server connection lost."
            else:
                 rospy.loginfo("Reconnected to Move Base server.")

        if self.navigation_active:
            rospy.logwarn("Attempted to navigate while already active.")
            return "Navigation failed: Already handling a navigation goal. Please wait."

        coords = locations.get_location_coords(location_name)
        if coords is None:
            clean_name = location_name.strip().upper()
            rospy.logerr(f"Location '{clean_name}' not found.")
            known_locs = locations.get_known_locations()
            return f"Navigation failed: Location '{clean_name}' is unknown. Known locations are: {', '.join(known_locs)}"

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(coords["x"], coords["y"], 0.0)
        # Use .get with default 0.0 for yaw, convert degrees to radians
        yaw_rad = np.radians(coords.get("yaw", 0.0))
        q = quaternion_from_euler(0, 0, yaw_rad)
        goal.target_pose.pose.orientation = Quaternion(*q)

        rospy.loginfo(f"Sending goal to /move_base and waiting: Go to {location_name.upper()} at ({coords['x']:.2f}, {coords['y']:.2f}, yaw={coords.get('yaw', 0.0):.1f}deg)")
        self.navigation_active = True

        try:
            self.move_base_client.send_goal(goal)
            # Increased timeout slightly, adjust as needed for your simulation/robot speed
            wait_success = self.move_base_client.wait_for_result(rospy.Duration(90.0)) # Wait for 90 seconds

            if not wait_success:
                rospy.logerr("Navigation goal did not complete within timeout (90s). Cancelling.")
                self.move_base_client.cancel_goal()
                self.navigation_active = False
                return "Navigation failed: Goal timed out."

            status = self.move_base_client.get_state()
            status_text = GoalStatus.to_string(status)
            rospy.loginfo(f"Navigation finished with status: {status_text} ({status})")

            if status == GoalStatus.SUCCEEDED:
                return f"Navigation successful: Reached {location_name.upper()}."
            elif status == GoalStatus.PREEMPTED:
                return "Navigation failed: Goal was preempted (cancelled)."
            elif status == GoalStatus.ABORTED:
                return "Navigation failed: Could not reach the goal (Aborted by move_base - check logs)."
            else:
                return f"Navigation failed: Ended with unhandled status {status_text}."

        except Exception as e:
             rospy.logerr(f"Exception while sending goal or waiting for result: {e}")
             return f"Navigation failed: Error during execution ({e})."
        finally:
             # Ensure navigation_active is reset even if errors occur
             self.navigation_active = False


# --- Main Execution Block ---
def main(args=None):
    # Initialize ROS node FIRST
    rospy.init_node('rosa_turtlebot_agent_node', anonymous=True)

    # Load environment variables (like API keys) AFTER node init if they affect ROS params potentially
    dotenv.load_dotenv(dotenv.find_dotenv())

    agent_instance = None # To ensure agent is defined for finally block

    try:
        # Create the agent instance (which now also creates the ActionClient)
        agent_instance = TurtleBotAgent(verbose=True, streaming=True) # Set streaming=False if astream causes issues

        # Run the main interaction loop
        asyncio.run(agent_instance.run())

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted by Ctrl+C or shutdown signal.")
    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupt received.")
    except Exception as e:
         rospy.logerr(f"An error occurred in the main loop: {e}")
         import traceback
         traceback.print_exc() # Print full traceback for debugging
    finally:
        rospy.loginfo("Exiting ROSA agent script.")
        # ROS 1 nodes usually clean up themselves on script exit/signal
        # No explicit rospy.shutdown() needed here unless for specific resource release

if __name__ == "__main__":
    main()