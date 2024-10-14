import os

# Base directory where the folders will be created
base_dir = 'boxes'

# Create 20 folders for box_0 to box_19
for i in range(20):
    folder_name = f'box_{i}'
    folder_path = os.path.join(base_dir, folder_name)
    
    # Create the box folder
    os.makedirs(folder_path, exist_ok=True)
    
    # Create the blank model.config file
    config_file_path = os.path.join(folder_path, 'model.config')
    with open(config_file_path, 'w') as f:
        f.write('')  # Blank file
    
    # Create the blank model.sdf file
    sdf_file_path = os.path.join(folder_path, 'model.sdf')
    with open(sdf_file_path, 'w') as f:
        f.write('')  # Blank file
    
    # Create the 'meshes' folder inside each box folder
    meshes_folder_path = os.path.join(folder_path, 'meshes')
    os.makedirs(meshes_folder_path, exist_ok=True)

'Folders created successfully!'
