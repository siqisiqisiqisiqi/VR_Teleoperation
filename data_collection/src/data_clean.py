import os
import shutil

# Define base path
base_path = "lerobot_dataset"

# List of folders to clean
folders_to_clean = [
    os.path.join(base_path, "data", "chunk-000"),
    os.path.join(base_path, "meta"),
    os.path.join(base_path, "videos", "chunk-000")
]

def clean_folder(folder_path):
    if not os.path.exists(folder_path):
        print(f"Folder not found: {folder_path}")
        return

    for filename in os.listdir(folder_path):
        file_path = os.path.join(folder_path, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.remove(file_path)
                print(f"Removed file: {file_path}")
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
                print(f"Removed folder: {file_path}")
        except Exception as e:
            print(f"Failed to delete {file_path}: {e}")

# Run cleaning
for folder in folders_to_clean:
    clean_folder(folder)