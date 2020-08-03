import os
from pathlib import Path

root_dir = Path(os.getcwd())/ '..' / '..' 
data_root = root_dir

print("Data Root : ", data_root)
ext = '.pkl'
input_size = 320
data_folder = data_root / 'training'

cacheFile = data_root / 'train_cache.ptar'
