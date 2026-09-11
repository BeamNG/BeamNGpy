"""Utilities for tech_ground level."""
from beamngpy import BeamNGpy


def add_simple_road_network(bng: BeamNGpy) -> None:
    """Add a simple road network to the tech ground level (requires reloading the level)."""
    bng._send(dict(type="AddTechGroundSimpleRoadNetwork")).ack("TechGroundSimpleRoadNetworkAdded")


def clear_tech_ground(bng: BeamNGpy) -> None:
    """Deletes all custom files from the tech ground level in the user folder."""
    bng._send(dict(type="ClearTechGround")).ack("TechGroundCleared")
