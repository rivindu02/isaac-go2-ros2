import argparse

# Parse args before SimulationApp absorbs them!
parser = argparse.ArgumentParser(description="Convert OBJ/PLY to USD")
parser.add_argument("--input", type=str, required=True, help="Path to input OBJ/PLY")
parser.add_argument("--output", type=str, required=True, help="Path to output USD")
args = parser.parse_args()

import asyncio
from omni.isaac.kit import SimulationApp

# Start the simulation app in headless mode
simulation_app = SimulationApp({"headless": True})

import omni.kit.asset_converter

done = False

async def convert_asset_async(input_path, output_path):
    global done
    print(f"Converting {input_path} to {output_path}...")
    
    # Configure converter options (generate physics, material handling)
    converter_context = omni.kit.asset_converter.AssetConverterContext()
    converter_context.ignore_materials = False
    converter_context.import_materials = True
    
    # Create the converter task
    task = omni.kit.asset_converter.get_instance().create_converter_task(
        input_path, 
        output_path, 
        None, 
        converter_context
    )
    
    success = await task.wait_until_finished()
    if not success:
        print(f"Error during conversion: {task.get_error_message()}")
    else:
        print(f"Successfully created: {output_path}")
        
    done = True
    return success

def main():
    asyncio.ensure_future(convert_asset_async(args.input, args.output))
    while simulation_app.is_running() and not done:
        simulation_app.update()
    
if __name__ == "__main__":
    main()
    simulation_app.close()
