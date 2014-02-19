#!/usr/bin/env python

import urllib2
import tarfile
import os

DOWNLOAD_URL = "https://dl.dropboxusercontent.com/s/r23dxeaahpobjuj/ModernMapOccupancyGrid.tgz"

LOCALFILENAME = 'ModernMapOccupancyGrid.tgz'

def scriptdir():
    return os.path.dirname(__file__) or './'

def download_untar(data_url, fname, parentdir):
    localfname = os.path.join(parentdir, fname)
    if not os.path.exists(localfname) or os.stat(localfname)[6] == 0:
        url = urllib2.urlopen(data_url)
        localFile = open(localfname, 'w')
        print 'Downloading test data:', url.geturl(), ' to ', parentdir + fname
        localFile.write(url.read())
        localFile.close()
    
    # extract tar ball
    tar = tarfile.open(localfname, mode='r:gz')
    tar.extractall(path=parentdir)
    tar.close()

    return os.path.splitext(localfname)[0]


if __name__ == '__main__':
    download_untar(DOWNLOAD_URL, LOCALFILENAME, scriptdir())
