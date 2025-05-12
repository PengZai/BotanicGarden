import os,sys

path = "/media/spiderman/zhipeng_8t/datasets/BotanicGarden/1005-07/imagezip/semantic_1005_07/07"
dirs = os.listdir(path)

for file in dirs:
    if os.path.splitext(file)[1] == ".tif":
        new = file[34:]
        os.rename(file,new)
        print(new)