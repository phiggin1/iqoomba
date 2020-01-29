import cv2
import numpy as np
import os
import argparse
import ntpath
import re
import json
import glob
from pprint import pprint

IMSIZE=(256,256)

# ~-~-~  ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ 
def process_comboimages(allfilesinfo, chandic, totchan, SAVEDIR):
	filewriteinfo = []
	count = 0
	ftotsize = len(allfilesinfo[0])
	imgtmp = np.zeros(  (IMSIZE[0],IMSIZE[1]*totchan), dtype='uint8' )
	for i in range(ftotsize):
		offc = 0
		for l in range(len(allfilesinfo)):
			tfile = allfilesinfo[l][i]
			img = cv2.imread(tfile, -1);
			if(len(img.shape)==2):
				newimg = scaleit_experimental(img)
			else:	
				newimg = scaleit3(img)
			if(len(img.shape)==2):
				newimg = cv2.applyColorMap(newimg, cv2.COLORMAP_JET)
			for c in range(newimg.shape[2]):
				imgtmp[:,offc:offc+IMSIZE[1]] = newimg[:,:,c]
				offc += IMSIZE[0]
		fname= ntpath.basename(tfile)
		outfile = SAVEDIR + '/' + os.path.splitext(fname)[0] + '_fus.png'
		outfile = os.path.abspath(outfile)
		cv2.imwrite(outfile, imgtmp)
		classname = tfile.split("/")[-1:][0]
		m = re.search("\d", classname)
		if m:
			namekey = getobjclasses()
			classnum = namekey[classname[:m.start()-1]]
			filewriteinfo.append( [outfile, classnum ] )
			count +=1	
		else:
			print("No digit in that string")
			return(-1)
	return (filewriteinfo)

# ~-~-~  ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ 
# The colorized depth image
# ~-~-~  ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ 
def scaleit_experimental(img):
	#img_mask = (img == 0)
	#istats = ( np.min(img[img>0]),	np.max(img))
	#imrange=  1.0-(img.astype('float32')-istats[0])/(istats[1]-istats[0]);
	#imrange[img_mask] = 0
	#imrange= 255.0*imrange
	imsz = img.shape
	mxdim  = np.max(imsz)

	offs_col = (mxdim - imsz[1])/2
	offs_row = (mxdim - imsz[0])/2	
	nchan = 1
	if(len(imsz)==3):
		nchan = imsz[2]
	imgcanvas = np.zeros(  (mxdim,mxdim,nchan), dtype='uint8' )
	imgcanvas[offs_row:offs_row+imsz[0], offs_col:offs_col+imsz[1]] = img.reshape( (imsz[0],imsz[1],nchan) )
	# take rows
	if(offs_row):
		tr = img[0,:]
		br = img[-1,:]
		imgcanvas[0:offs_row,:,0] = np.tile(tr, (offs_row,1))
		imgcanvas[-offs_row-1:,:,0] = np.tile(br, (offs_row+1,1))

	# take cols
	if(offs_col):
		lc = img[:,0]
		rc = img[:,-1]
		imgcanvas[:, 0:offs_col,0] = np.tile(lc, (offs_col,1)).transpose()
		imgcanvas[:, -offs_col-1:,0] = np.tile(rc, (offs_col+1,1)).transpose()

	# RESCALE
	imrange_rescale = cv2.resize(imgcanvas, IMSIZE, interpolation=cv2.INTER_CUBIC) 
	return(imrange_rescale)

# ~-~-~  ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ 
def scaleit3(img):
 	
	imsz = img.shape

	# RESCALE
	rd = float(IMSIZE[0])/float(np.max(imsz))
	imrange_rescale = cv2.resize(img, (int(rd*imsz[1]), int(rd*imsz[0])), interpolation=cv2.INTER_CUBIC) 
	imszn = imrange_rescale.shape
	# print imszn
	nchan = 1
	if(len(imszn)==3):
		nchan = imszn[2]

	# FILL IT
	imgcanvas = np.zeros(  (IMSIZE[0],IMSIZE[1],nchan), dtype='uint8' )
	offs_col = (IMSIZE[1] - imszn[1])/2
	offs_row = (IMSIZE[0] - imszn[0])/2

	print(offs_col, offs_row)

	# take cols
	imgcanvas[offs_row:offs_row+imszn[0], offs_col:offs_col+imszn[1]] = imrange_rescale.reshape( (imszn[0],imszn[1],nchan) )
	return (imgcanvas)

# ~-~-~  ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ 

def getobjclasses():
    namekey = {}
    with open('../data/namedict.json') as data_file:    
        namekey = json.load(data_file)
    return namekey[1]

# ~-~-~  ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ 
# ~-~-~  ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ 
# ~-~-~  ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ ~-~-~ 

def fuse_color_depth(data_dir = '../data'):
	output_dir =  os.path.abspath('../data/fus')
	chandic =[ [0,3],[3,6] ]
	num_channels = 6
	rgb_im_paths = sorted(glob.glob('../data/rgb/*.png'))
	depth_im_paths = sorted(glob.glob('../data/depth/*.png'))
	assert len(rgb_im_paths) == len(depth_im_paths), \
             'Image pairs in ../data/rgb and ../data/depth must match'
	combo_path = [rgb_im_paths, depth_im_paths]

	namemap = getobjclasses()
	file_info = process_comboimages(combo_path, chandic, num_channels, output_dir) 
	print("Number of images processed:", len(file_info) )
	oname = output_dir+'/testlist.txt'
	file = open(oname, "w")
	for fnm in file_info :
		file.write(fnm[0].strip("/") +' %d\n'  % (fnm[1]))
	file.close()
	print("Wrote ", oname)
