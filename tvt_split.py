import os
import random
import shutil

source_dir = 'posecard' + '/'
dest_dir = 'dataset' + '/'

full = os.listdir(source_dir)
val_sz = 0.2
random.seed(3)
random.shuffle(full)

val = full[:int(val_sz * len(full))]
train = full[int(val_sz * len(full)):]

for e in val:
    shutil.copy(source_dir + e, dest_dir + 'val/' + e)
for e in train:
    shutil.copy(source_dir + e, dest_dir + 'train/' + e)
