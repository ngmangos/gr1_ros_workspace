from isaacsim import SimulationApp

# Simple example showing how to start and stop the helper
simulation_app = SimulationApp({"headless": False})

### Perform any omniverse imports here after the helper loads ###

simulation_app.update()  # Render a single frame
simulation_app.close()  # Cleanup application