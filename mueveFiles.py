import glob, os, sys
from os import system
import re

new = sys.argv[1]
old = sys.argv[2]
path_new = "{}/*.jpg".format(new)
path_old = "{}/*.jpg".format(old)

f_nuevos = [os.path.basename(x) for x in glob.glob(path_new)]
f_viejos = [os.path.basename(x) for x in glob.glob(path_old)]

temp_dir = old + "/.temp"
system("mkdir " + temp_dir)

for filename in f_viejos:
	a = 2
	system("mv " + old + "/" + filename + " " + temp_dir + "/" + filename)

for filename in f_nuevos:
	filename_new = filename
	if (filename in f_viejos):
		filename_new = filename[:-4] + "0000000.jpg"
	
	system("mv {}/{} {}/{}".format(new, filename, temp_dir, filename_new))

all_files = glob.glob(temp_dir + "/*.jpg")
i = 0
for path in all_files:
	filename = os.path.basename(path)
	regex = r"([A-Z_]+)_\d+\.jpg"
	category = re.findall(regex, filename, re.MULTILINE)[0]
	new_filename = category + "_" + str(i) + ".jpg"
	system("mv " + path + " " + old + "/" + new_filename)
	i += 1

system("rm -rf " + temp_dir)