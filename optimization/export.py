"""Export the same parameterized base mechanism used for optimization tests."""

from .model_factory import build_template_model


def export_model(candidate, inputs, requirement):
    """Return a complete actuated MJCF; unsupported base geometry is rejected."""
    _, xml = build_template_model(candidate, inputs, actuated=True, requirement=requirement)
    return xml
