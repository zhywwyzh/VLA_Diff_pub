import pandas as pd
from PIL import Image
import io
import os
from tqdm import tqdm

def process_parquet_file(parquet_path, output_dir):
    """
    Processes a single Parquet file to extract the first image of each episode.

    Args:
        parquet_path (str): The full path to the Parquet file.
        output_dir (str): The directory where images for the category should be saved.
    """
    try:
        df = pd.read_parquet(parquet_path)
    except Exception as e:
        print(f"Error reading {parquet_path}: {e}")
        return

    if df.empty:
        print(f"Warning: {parquet_path} is empty.")
        return

    # Sort by id and frame_idx to ensure we get the first frame correctly
    df.sort_values(by=['id', 'frame_idx'], inplace=True)
    
    # Group by 'id' and take the first entry of each group
    first_frames_df = df.groupby('id').first().reset_index()

    parquet_filename_base = os.path.splitext(os.path.basename(parquet_path))[0]

    for index, row in first_frames_df.iterrows():
        episode_id = row['id']
        image_data = row['image']
        
        image_bytes = None
        if isinstance(image_data, dict) and 'bytes' in image_data:
            image_bytes = image_data['bytes']
        elif isinstance(image_data, bytes):
            image_bytes = image_data
        
        if image_bytes:
            try:
                image = Image.open(io.BytesIO(image_bytes))
                # Define a unique name for each image
                image_filename = f"{parquet_filename_base}_episode_{episode_id}.png"
                image_filepath = os.path.join(output_dir, image_filename)
                image.save(image_filepath)
            except Exception as e:
                print(f"Could not save image for episode {episode_id} from {parquet_path}: {e}")
        else:
            print(f"No image bytes found for the first frame of episode {episode_id} in {parquet_path}.")


def main():
    """
    Main function to walk through directories and process files.
    """
    input_base_dir = '/data/vla/uav_flow_raw_categorized'
    # Define a new output directory to store categorized images
    output_base_dir = '/data/vla/Openpi/test/parquet_2_json/uav_flow/categorized_images'

    print(f"Input directory: {input_base_dir}")
    print(f"Output directory: {output_base_dir}")

    if not os.path.isdir(input_base_dir):
        print(f"Error: Input directory '{input_base_dir}' not found.")
        return

    # Get category subdirectories
    categories = [d for d in os.listdir(input_base_dir) if os.path.isdir(os.path.join(input_base_dir, d))]

    for category in tqdm(categories, desc="Processing Categories"):
        category_input_path = os.path.join(input_base_dir, category)
        category_output_path = os.path.join(output_base_dir, category)

        # Create corresponding output directory for the category
        os.makedirs(category_output_path, exist_ok=True)
        
        print(f"\nProcessing category: {category}")

        parquet_files = [f for f in os.listdir(category_input_path) if f.endswith('.parquet')]
        
        for parquet_file in tqdm(parquet_files, desc=f"  Files in {category}", leave=False):
            parquet_full_path = os.path.join(category_input_path, parquet_file)
            process_parquet_file(parquet_full_path, category_output_path)

    print("\nProcessing complete.")

if __name__ == '__main__':
    main()