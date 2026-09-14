"""Robot Registry and Model Discovery Service.

Manages registration and instantiation of supported manipulators and their
corresponding kinematics adapters.
"""

from typing import Dict, List, Optional, Type
from robotics.robot_model import RobotModelSpec
from robotics.adapters.base import RobotAdapter
from robotics.adapters.panda import PandaAdapter
from robotics.adapters.kuka_iiwa import KukaIiwaAdapter


class RobotRegistry:
    """Registry maintaining supported robotic manipulators and their factories."""

    def __init__(self):
        self._specs: Dict[str, RobotModelSpec] = {}
        self._adapter_classes: Dict[str, Type[RobotAdapter]] = {}
        self._register_defaults()

    def _register_defaults(self) -> None:
        """Registers out-of-the-box supported robots."""
        panda_adapter = PandaAdapter()
        self.register_robot(panda_adapter.spec, PandaAdapter)

        kuka_adapter = KukaIiwaAdapter()
        self.register_robot(kuka_adapter.spec, KukaIiwaAdapter)

    def register_robot(self, spec: RobotModelSpec, adapter_cls: Type[RobotAdapter]) -> None:
        """Registers a new robot specification and adapter class."""
        self._specs[spec.robot_id] = spec
        self._adapter_classes[spec.robot_id] = adapter_cls

    def list_robot_ids(self) -> List[str]:
        """Returns the list of all registered robot IDs."""
        return list(self._specs.keys())

    def get_robot_spec(self, robot_id: str) -> RobotModelSpec:
        """Retrieves the RobotModelSpec for the given robot_id.
        
        Raises:
            ValueError: If the robot is not registered.
        """
        if robot_id not in self._specs:
            avail = ", ".join(self.list_robot_ids())
            raise ValueError(f"Unsupported robot '{robot_id}'. Available robots: {avail}")
        return self._specs[robot_id]

    def create_adapter(self, robot_id: str, custom_spec: Optional[RobotModelSpec] = None) -> RobotAdapter:
        """Instantiates the adapter for the given robot ID."""
        spec = custom_spec or self.get_robot_spec(robot_id)
        adapter_cls = self._adapter_classes[robot_id]
        return adapter_cls(spec)


# Global singleton instance
_GLOBAL_REGISTRY: Optional[RobotRegistry] = None


def get_robot_registry() -> RobotRegistry:
    """Returns the singleton RobotRegistry instance."""
    global _GLOBAL_REGISTRY
    if _GLOBAL_REGISTRY is None:
        _GLOBAL_REGISTRY = RobotRegistry()
    return _GLOBAL_REGISTRY


def list_available_robots() -> List[str]:
    """Convenience function listing all available robot IDs."""
    return get_robot_registry().list_robot_ids()


def create_robot_adapter(robot_id: str, custom_spec: Optional[RobotModelSpec] = None) -> RobotAdapter:
    """Convenience function to create a robot adapter."""
    return get_robot_registry().create_adapter(robot_id, custom_spec)
