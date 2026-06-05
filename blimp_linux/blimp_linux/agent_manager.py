import threading
import traceback
import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from blimp_linux.low_level_controller import ControllerNode

class AgentManager(object):
    def __init__(self):
        self.executor = MultiThreadedExecutor()
        self.agents = {}
        self._thread = None

    def initialize_blimps(self, ids, coms, goals):
        # Tear down any previously running agents so we can safely re-apply
        self._stop_existing()
        for id_, com in zip(ids, coms):
            self.spawn_agent(id_, com)
        self.start()  # spin only after all blimps are registered

    def _stop_existing(self):
        """Shut down running agents and reset executor so initialize can be called again."""
        if self.agents:
            self.executor.shutdown(timeout_sec=1.0)
            for node in self.agents.values():
                node.destroy_node()
            self.agents.clear()
            self.executor = MultiThreadedExecutor()
            self._thread = None

    def spawn_agent(self, agent_id: int, com_port: str):
        node = ControllerNode(f'agent_{agent_id}', com_port)
        self.agents[agent_id] = node
        self.executor.add_node(node)

    def destroy_agent(self, agent_id: str):
        node = self.agents.pop(agent_id)
        self.executor.remove_node(node)
        node.destroy_node()

    def start(self):
        """Start executor in background thread after blimps are initialized"""
        print("Starting executor")
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()

    def _spin_loop(self):
        """Spin the executor and keep spinning if a callback raises.

        Without this wrapper, an exception in any subscriber callback propagates
        out of executor.spin(), kills the daemon thread silently, and makes every
        ControllerNode stop responding a few seconds later.
        """
        while rclpy.ok() and self.agents:
            try:
                self.executor.spin()
                # spin() returned normally -> shutdown was requested
                return
            except ExternalShutdownException:
                return
            except Exception:
                traceback.print_exc()

    def shutdown(self):
        """Clean up all nodes and stop executor"""
        self.executor.shutdown()
        for node in self.agents.values():
            node.destroy_node()
        self.agents.clear()
        rclpy.shutdown()