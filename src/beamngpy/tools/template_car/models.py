"""Definition of model classes."""

from pydantic import BaseModel, Field
from enum import Enum
import numpy as np
from typing import Literal, Optional


class Settings(BaseModel):
    user_folder: str = Field(
        title="BeamNG user folder path",
        description='Leave blank for default. Don\'t append "current" because that\'s added automatically.',
        default="",
    )
    vehicles_folder: str = Field(
        title="Template vehicles folder path",
        description="Path to template vehicle configurations. Leave blank for default.",
        default="",
    )
    beamng_install_path: str = Field(
        title="BeamNG installation path",
        description="Path to the BeamNG installation folder. Leave blank for default.",
        default="",
    )


class VehicleListItem(BaseModel):
    id: str = Field(title="Vehicle ID")
    name: str = Field(title="Vehicle name")
    is_generated: bool = Field(title="True if the mod folder exists", default=False)


class Message(BaseModel):
    message: str = Field(title="Response message")


class VehicleParameters(BaseModel):
    wheelbase: float = Field(title="Wheelbase", description="unit: m", default=2.52, ge=1.0, le=10.0)
    track_width_front: float = Field(title="Track width front", description="unit: m", default=1.40, ge=0.80, le=3.0)
    track_width_rear: float = Field(title="Track width rear", description="unit: m", default=1.40, ge=0.80, le=3.0)
    front_overhang: float = Field(title="Front overhang", description="unit: m", default=0.68, ge=0.3, le=2.0)
    rear_overhang: float = Field(title="Rear overhang", description="unit: m", default=0.70, ge=0.3, le=2.0)
    body_width: float = Field(title="Body width", description="unit: m", default=1.60, ge=1.20, le=3.2)
    body_height: float = Field(title="Body height", description="unit: m", default=1.32, ge=0.8, le=4.2)
    ride_height: float = Field(title="Ride height", description="unit: m", default=0.105, ge=-0.5, le=0.5)
    front_spring_rate: float = Field(title="Front spring rate", description="unit: N/m", default=27465, ge=100, le=100000)
    front_damping: float = Field(title="Front damping", description="unit: N/m/s", default=5708, ge=10, le=100000)
    rear_spring_rate: float = Field(title="Rear spring rate", description="unit: N/m", default=33609, ge=100, le=100000)
    rear_damping: float = Field(title="Rear damping", description="unit: N/m/s", default=6362, ge=10, le=100000)
    front_swaybar_spring_rate: float = Field(title="Front sway bar spring rate", description="unit: Nm/rad", default=1200, ge=10, le=10000)
    rear_swaybar_spring_rate: float = Field(title="Rear sway bar spring rate", description="unit: Nm/rad", default=500, ge=10, le=10000)


class BodyShape(str, Enum):
    """Body shape (Enum)."""
    SEDAN = "sedan"
    WAGON = "wagon"
    COUPE = "coupe"
    PICKUP = "pickup"


class SuspensionFront(str, Enum):
    """Suspension front (Enum)."""
    DOUBLEWISHBONE = "doublewishbone"
    LIVEAXLE_4LINK = "liveaxle_4link"
    STRUT = "strut"


class SuspensionRear(str, Enum):
    """Suspension rear (Enum)."""
    DOUBLEWISHBONE = "doublewishbone"
    LIVEAXLE_4LINK = "liveaxle_4link"
    LIVEAXLE_4LINK_DUALLY = "liveaxle_4link_dually"
    STRUT = "strut"
    TORSIONBEAM = "torsionbeam"
    SEMITRAILING = "semitrailing"
    MULTILINK = "multilink"


class VehicleStructure(BaseModel):
    body_shape: BodyShape = Field(title="Body shape", default=BodyShape.SEDAN)
    suspension_front: SuspensionFront = Field(title="Front suspension", default=SuspensionFront.DOUBLEWISHBONE)
    suspension_rear: SuspensionRear = Field(title="Rear suspension", default=SuspensionRear.DOUBLEWISHBONE)


class BaseOptimizationEnabled(BaseModel):
    pass


class BaseVariables(BaseModel):
    def _get_enabled(self, enabled: BaseOptimizationEnabled, extractor) -> list:
        values = []
        for name, is_enabled in enabled.model_dump().items():
            n = name if isinstance(self, BaseTargets) else f"{name}_factor"
            if is_enabled:
                values.append(extractor(self, n))
        return values

    def _get_bound(self, field_name: str, bound_type: Literal["ge", "le"]) -> Optional[float]:
        constraints = type(self).model_fields[field_name].metadata
        for c in constraints:
            if hasattr(c, bound_type):
                return getattr(c, bound_type)
        return None
    
    def _get_extra(self, field_name: str, key: str):
        return type(self).model_fields[field_name].json_schema_extra[key]

    def min_step(self, enabled: BaseOptimizationEnabled) -> np.ndarray:
        return np.array(self._get_enabled(enabled, lambda s, name: s._get_extra(name, 'min_step')), dtype=np.float64)

    def tolerance(self, enabled: BaseOptimizationEnabled) -> np.ndarray:
        return np.array(self._get_enabled(enabled, lambda s, name: s._get_extra(name, 'tolerance')), dtype=np.float64)

    def min_values(self, enabled: BaseOptimizationEnabled) -> np.ndarray:
        return np.array(self._get_enabled(enabled, lambda s, name: s._get_bound(name, 'ge')), dtype=np.float64)

    def max_values(self, enabled: BaseOptimizationEnabled) -> np.ndarray:
        return np.array(self._get_enabled(enabled, lambda s, name: s._get_bound(name, 'le')), dtype=np.float64)

    def to_numpy(self, enabled: BaseOptimizationEnabled) -> np.ndarray:
        """Convert the variables to a numpy array for optimization."""
        return np.array(self._get_enabled(enabled, lambda s, name: getattr(s, name)), dtype=np.float64)

    def from_numpy(self, arr: np.ndarray, enabled: BaseOptimizationEnabled):
        """Convert a numpy array from optimization to the variables object."""
        d = {}
        enabled_values = enabled.model_dump()
        index = 0
        for name, is_enabled in enabled_values.items():
            if is_enabled:
                value = float(arr[index])
                index += 1
            else:
                value = getattr(self, f"{name}_factor")
            d[f"{name}_factor"] = value
        return self.model_validate(d)


class BaseTargets(BaseVariables):
    pass


class OptimizationEnabled(BaseOptimizationEnabled):
    mass: bool = Field(default=False)
    cg_y: bool = Field(default=False)
    cg_z: bool = Field(default=False)
    inertia_yaw: bool = Field(default=False)
    inertia_pitch: bool = Field(default=False)
    inertia_roll: bool = Field(default=False)


class OptimizationVariables(BaseVariables):
    mass_factor: float = Field(
        title="Mass factor", description="", default=1.0, ge=0.0, le=10.0, json_schema_extra={"min_step": 1e-4}
    )
    cg_y_factor: float = Field(
        title="Center of gravity longitudinal (y) factor", description="", default=0.0, ge=-10.0, le=10.0, json_schema_extra={"min_step": 1e-3}
    )
    cg_z_factor: float = Field(
        title="Center of gravity height (z) factor", description="", default=0.0, ge=-10.0, le=10.0, json_schema_extra={"min_step": 1e-3}
    )
    inertia_yaw_factor: float = Field(
        title="Inertia yaw factor", description="", default=0.0, ge=-10.0, le=10.0, json_schema_extra={"min_step": 1e-3}
    )
    inertia_pitch_factor: float = Field(
        title="Inertia pitch factor", description="", default=0.0, ge=-10.0, le=10.0, json_schema_extra={"min_step": 1e-3}
    )
    inertia_roll_factor: float = Field(
        title="Inertia roll factor", description="", default=0.0, ge=-10.0, le=10.0, json_schema_extra={"min_step": 1e-3}
    )


class TargetValues(BaseTargets):
    mass: float = Field(
        title="Mass", description="unit: kg", default=1500, ge=500, le=5000, json_schema_extra={"tolerance": 0.01}
    )
    cg_y: float = Field(
        title="Center of gravity longitudinal (y)", description="unit: m", default=1.3, ge=0.0, le=5.0, json_schema_extra={"tolerance": 1e-3}
    )
    cg_z: float = Field(
        title="Center of gravity height (z)", description="unit: m", default=0.25, ge=0.0, le=5.0, json_schema_extra={"tolerance": 1e-3}
    )
    inertia_yaw: float = Field(
        title="Yaw Inertia", description="unit: kg m^2", default=1500, ge=800, le=9000, json_schema_extra={"tolerance": 1e-3}
    )
    inertia_pitch: float = Field(
        title="Pitch Inertia", description="unit: kg m^2", default=1500, ge=800, le=9000, json_schema_extra={"tolerance": 1e-3}
    )
    inertia_roll: float = Field(
        title="Roll Inertia", description="unit: kg m^2", default=400, ge=150, le=1500, json_schema_extra={"tolerance": 1e-3}
    )


class OutputValues(BaseModel):
    mass: float = Field(title="Mass", description="unit: kg")
    cg_y: float = Field(title="Center of gravity longitudinal (y)", description="unit: m")
    cg_z: float = Field(title="Center of gravity height (z)", description="unit: m")
    inertia_yaw: float = Field(title="Yaw Inertia", description="unit: kg m^2")
    inertia_pitch: float = Field(title="Pitch Inertia", description="unit: kg m^2")
    inertia_roll: float = Field(title="Roll Inertia", description="unit: kg m^2")


class Optimization(BaseModel):
    variables: OptimizationVariables = Field(title="Optimization variables", default=OptimizationVariables())
    targets: TargetValues = Field(title="Optimization targets", default=TargetValues())
    enabled: OptimizationEnabled = Field(title="Optimization enabled", default=OptimizationEnabled())

    def required(self) -> bool:
        """Return True if optimization is required (at least one enabled field is set to True)."""
        for field_name in type(self.enabled).model_fields.keys():
            if getattr(self.enabled, field_name):
                return True
        return False


class VehicleInputs(BaseModel):
    """Input values needed for generation of a vehicle mod."""
    id: str = Field(title="ID")
    name: str = Field(title="Name")
    structure: VehicleStructure = Field(title="Structure")
    parameters: VehicleParameters = Field(title="Parameters")
    variables: OptimizationVariables = Field(title="Optimization variable values")


class TemplateVehicle(BaseModel):
    """Template car user parameters."""
    name: str = Field(title="Name")
    structure: VehicleStructure = Field(title="Structure", default=VehicleStructure())
    parameters: VehicleParameters = Field(title="Parameters", default=VehicleParameters())
    optimization: Optimization = Field(title="Target value optimization", default=Optimization())

    def to_inputs(self, id: str, var: OptimizationVariables) -> VehicleInputs:
        return VehicleInputs(
            id = id,
            name = self.name,
            structure = self.structure,
            parameters = self.parameters,
            variables = var,
        )
