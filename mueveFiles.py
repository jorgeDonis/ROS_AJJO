import glob, os, sys

new = sys.argv[1]
old = sys.argv[2]
path_new = "{}/*.jpg".format(new)
path_old = "{}/*.jpg".format(old)


f_nuevos = [os.path.basename(x) for x in glob.glob(path_new)]
f_viejos = [os.path.basename(x) for x in glob.glob(path_old)]

for filename in f_nuevos:
	filename_new = filename
	if (filename in f_viejos):
		filename_new = filename[:-4] + "00000.jpg"
	
	os.system("mv {}/{} {}/{}".format(new, filename, old, filename_new))

