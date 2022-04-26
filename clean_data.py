import os
from glob import glob

for i in glob('./data/gray_before/*'):
    os.remove(i)
    
for i in glob('./data/gray_after/*'):
    os.remove(i)