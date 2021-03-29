import glob, os


f_nuevos = [os.path.basename(x) for x in glob.glob("nuevo/*.jpg")]
f_viejos = [os.path.basename(x) for x in glob.glob("VueltasAlex/*.jpg")]

for filename in f_nuevos:
	filename_new = filename
	if (filename in f_viejos):
		filename_new = filename[:-4] + "00000.jpg"
	
	os.system("cp nuevo/{} VueltasAlex/{}".format(filename, filename_new))

