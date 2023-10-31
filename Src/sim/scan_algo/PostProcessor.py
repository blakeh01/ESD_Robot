import numpy as np
from scipy.interpolate import interp1d
from scipy.interpolate import RegularGridInterpolator
from scipy.fft import fft2
from scipy.fft import fftshift
from scipy.fft import ifftshift
from scipy.fft import ifft2
from scipy.interpolate import griddata

# Functions for post-processing of probe data to get surface charge density
# usage:
# a, sigma, gamma = gen_psf(2.1, 'PSF.csv')
# arrangeddata, ravg, thetastart, thetaend, zstart, zend = arrange_cylinder(indata, 100, 50) # indata: captured data
# processed = process_cylinder(thetastart, thetaend, zstart, zend, ravg, arrangeddata, a, sigma, gamma, 0.02)
# or:
# arrangeddata, xstart, xend, zstart, zend = arrange_face(indata, 50, 25) # indata: captured data
# processed = process_rect(xstart, xend, zstart, zend, arrangeddata, a, sigma, gamma, 0.02)


# creates a circular mask in the center of an array, for filtering
def create_circular_mask(h, w, center=None, radius=None):
    if center is None:  # use the middle of the image
        center = (int(w / 2), int(h / 2))
    if radius is None:  # use the smallest distance between the center and image walls
        radius = min(center[0], center[1], w - center[0], h - center[1])

    Y, X = np.ogrid[:h, :w]
    dist_from_center = np.sqrt((X - center[0]) ** 2 + (Y - center[1]) ** 2)

    mask = dist_from_center <= radius
    return mask


# used for padding arrays to a certain size around the center
def padding(array, xx, yy):
    """
    :param array: numpy array
    :param xx: desired height
    :param yy: desirex width
    :return: padded array
    """

    h = array.shape[0]
    w = array.shape[1]

    a = (xx - h) // 2
    aa = xx - a - h

    b = (yy - w) // 2
    bb = yy - b - w

    return np.pad(array, pad_width=((a, aa), (b, bb)), mode='linear_ramp')


# takes unordered r, theta, z, v (columns) data in and converts to theta, z, v array for further processing
def arrange_cylinder(indata, interpnumx=100, interpnumy=50):
    # input data in r, theta, z, v
    # needs to be in rectangular array with defined size (x, y) from input data
    # r: use average
    # theta: use as is (for compatibility with processing function)
    # z: use as is
    # interpnumx/y: not too critical, but make larger than input array by about 2x.
    r = indata[:, 0]
    theta = indata[:, 1]
    z = indata[:, 2]
    v = indata[:, 3]

    # get average radius
    # we don't need to know if there's any offset since the average of the r values will always be the radius of the cylinder
    ravg = np.mean(r)

    # circules are 0 to 360 degrees...
    thetastart = 0
    thetaend = 360
    zstart = min(z) # get start and end z values from the data
    zend = max(z)

    thetanew = np.linspace(thetastart, thetaend, interpnumx) # make linspaces for theta and z
    znew = np.linspace(zstart, zend, interpnumy)

    xv, yv = np.meshgrid(thetanew, znew) # 2D grid to fill out with data

    newgrid = griddata((theta, z), v, (xv, yv), method='cubic', fill_value=0) # takes unordered data and interpolates it onto the 2D grid

    return newgrid, ravg, thetastart, thetaend, zstart, zend


# takes unordered x, z, v (columns) data in and arranges into x, z, v array for further processing
def arrange_face(indata, interpnumx=100, interpnumy=100):
    # very similar to arrange_cylinder but with x y instead of r, theta, z
    # input data in x, y, v
    # x: use as is
    # y: use as is
    # interpnumx/y: not too critical, but make larger than input array by about 2x.
    x = indata[:, 0]
    y = indata[:, 1]
    v = indata[:, 2]

    # get limits of data in x and y
    xstart = min(x)
    xend = max(x)
    ystart = min(y)
    yend = max(y)
    # make linear spaces to interp onto
    xnew = np.linspace(xstart, xend, interpnumx)
    ynew = np.linspace(ystart, yend, interpnumy)

    xv, yv = np.meshgrid(xnew, ynew) # 2D grid to interp onto

    newgrid = griddata((x, y), v, (xv, yv), method='linear', fill_value=0) # takes unordered data and interpolates it onto the 2D grid

    return newgrid, xstart, xend, ystart, yend

# takes 'vase' unordered r, theta, z, v (columns) data in and converts to theta, z, v array for further processing
# use process_cylinder but pass rmax instead of ravg
def arrange_vase(indata, offsetx=0, offsety=0, interpnumx=100, interpnumy=50):
    # input data in r, theta, z, v from not a cylinder but a vase
    # this really just takes the theta/v data and flattens it, need to come up with a way to un-distort the data
    # needs to be in rectangular array with defined size (x, y) from input data
    # r: use max (assuming vase is centered)
    # theta: use as is (for compatibility with processing function)
    # z: use as is
    # interpnumx/y: not too critical, but make larger than input array by about 2x.

    # code is basically the same as arrange_cylinder but has a way to remove an offset and also returns the maximum radius
    r = indata[:, 0]
    theta = indata[:, 1]
    z = indata[:, 2]
    v = indata[:, 3]

    # convert to cartesian:
    xp = r * np.cos(theta * (np.pi / 180))
    yp = r * np.sin(theta * (np.pi / 180))

    # remove x and y offset
    xn = xp - offsetx
    yn = yp - offsety

    # back to polar:
    r = xn**xn + yn**yn
    theta = np.arctan(yn/xn)

    ravg = np.mean(r)
    rmax = np.max(r)

    thetastart = 0
    thetaend = 360
    zstart = min(z)
    zend = max(z)

    thetanew = np.linspace(thetastart, thetaend, interpnumx)
    znew = np.linspace(zstart, zend, interpnumy)

    xv, yv = np.meshgrid(thetanew, znew)

    newgrid = griddata((theta, z), v, (xv, yv), method='cubic', fill_value=0)

    return newgrid, ravg, rmax, thetastart, thetaend, zstart, zend


# takes data from rectangular face (previously arranged) and returns surface charge density
def process_rect(xmin, xmax, zmin, zmax, inmap, a, sig, g, mask_perc):
    # xmin, xmax: x limites of scanned part of face
    # zmin, zmax: limits of scanned part of face (can start at 0)
    # inmap: measured data in matrix form
    # sig: PSF
    # g: PSF
    # a: PSF
    # mask_perc: mask percentage - how large to make circular mask as a percentage of data size

    z = np.linspace(zmin, zmax, np.size(inmap[:, 0]))  # linspace for z of measured output
    x = np.linspace(xmin, xmax, np.size(inmap[0, :]))  # phi linspace for x of measured output

    xPSF = np.linspace(-600, 600, num=10000, endpoint=True)  # PSF x range (larger than needs to be)
    bigG = (a / (sig * np.sqrt(2 * np.pi))) * np.exp(-(np.square(xPSF)) / (2 * np.square(sig))) # Gaussian profile
    L = (g / np.pi) / (np.abs(xPSF) + np.square(g)) # Lorentz Profile

    vPSF = np.convolve(bigG, L) # convolve the two
    xPSF = np.linspace(-600, 600, 2 * np.size(xPSF) - 1, endpoint=True) # make new linspace for x

    dz = (zmax / np.size(inmap[:, 1])) / 4 # get dz from input, divide by 4
    dx = (np.max(x) / np.size(x)) / 4 # get dx from input, divide by 4
    xx, yy = np.mgrid[-400:400 + dx:dx, -400:400 + dz:dz]  # 2D grid to interpolate onto, -400 to 400 hopefully big enough
    # some padding room
    newGrid = np.sqrt(np.square(xx) + np.square(yy)) # the grid we will be interpolating onto, this makes a circular coordinate array

    f = interp1d(xPSF, vPSF) # interpolation function with x and v from above
    yPSF = f(newGrid)  # the big PSF we want

    [X, Y] = np.mgrid[0:np.max(x):dx, zmin:zmax + dz:dz] # a grid based on the data size and the new dx and dz

    interp2 = RegularGridInterpolator((z, x), inmap, bounds_error=False, fill_value=None) # interpolate the data
    inmap_prime = interp2((Y, X))

    # get the amount of padding we need
    xpad = yPSF.shape[0]
    ypad = yPSF.shape[1]
    inmap_padded = padding(inmap_prime, xpad, ypad) # pad the input data to the size of the PSF

    # the actual math:
    PS_FFT = fft2(yPSF) # get stuff into the frequency domain
    v_FFT = fft2(inmap_padded)

    # shift so FFT is zero centered
    PS_FFT = fftshift(PS_FFT)
    v_FFT = fftshift(v_FFT)

    M, N = np.shape(v_FFT) # get shape of the FFT

    # filter the FFT of the input data with a circular cookie cutter to remove high frequency noise:
    mask_radius = mask_perc * M # circle is a percentage of the size of the FFT
    mask = create_circular_mask(M, N, radius=mask_radius)
    v_FFT = v_FFT * mask

    # Wiener Deconvolution:
    H = PS_FFT
    Hdiv = (np.conj(H) / (np.square(np.abs(H)) + 8)) # 8 is an assumed noise level, 8 seems to work pretty well
    PHI = v_FFT

    wnrdeconv = Hdiv * PHI
    # now back to the spatial domain:
    sigma = ifftshift(wnrdeconv) # 'unshift' the data
    sigma_fft = ifft2(sigma) # inverse FFT
    sigma_plot = ifftshift(sigma_fft) # shift back to center

    # cut out the data in the center of the output array
    r_0, c_0 = np.asarray(inmap_prime.shape) # get start and end coordinates for where the data actually is
    r_1, c_1 = np.asarray(sigma_plot.shape)
    outmap = np.transpose(
        10 * np.real(sigma_plot[r_1 // 2 - r_0 // 2:r_1 // 2 + r_0 // 2, c_1 // 2 - c_0 // 2:c_1 // 2 + c_0 // 2])) # cut out the data and return
    return outmap

# takes data from cylinder (previously arranged) and returns surface charge density
def process_cylinder(phimin, phimax, zmin, zmax, r, inmap, a, sig, g, mask_perc):
    # phimin, phimax: usually 0 and 360
    # zmin, zmax: limits of scanned part of cylinder (can start at 0)
    # r: radius of cylinder
    # inmap: measured data in matrix form
    # sig: PSF
    # g: PSF
    # fit: PSF
    # mask_perc: mask percentage - how large to make circular mask as a percentage of data size

    # not as many comments on this one but very similar to process_rect(*) just with an angle and arclength

    nx, ny = np.shape(inmap)
    xx = np.linspace(phimin, phimax, ny)
    yy = np.linspace(zmin, zmax, nx)
    x, y = np.meshgrid(xx, yy)

    z = np.linspace(zmin, zmax, np.size(inmap[:, 0]))  # linspace for z of measured output
    phi = np.linspace(phimin, phimax, np.size(inmap[0, :]))  # phi linspace for phi of measured output
    arcl = r * phi * (np.pi / 180)
    xmin = min(arcl)
    xmax = max(arcl)

    xPSF = np.linspace(-600, 600, num=10000, endpoint=True)  # PSF x range (larger than needs to be)
    bigG = (a / (sig * np.sqrt(2 * np.pi))) * np.exp(-(np.square(xPSF)) / (2 * np.square(sig)))
    L = (g / np.pi) / (np.abs(xPSF) + np.square(g))

    vPSF = np.convolve(bigG, L)
    xPSF = np.linspace(-600, 600, 2 * np.size(xPSF) - 1, endpoint=True)

    dz = (zmax / np.size(inmap[:, 1])) / 4
    dx = (np.max(arcl) / np.size(arcl)) / 4
    xx, yy = np.mgrid[-400:400 + dx:dx, -400:400 + dz:dz]  # -400 to 400 to encompass an 8" diameter cylinder with
    # some padding room
    newGrid = np.sqrt(np.square(xx) + np.square(yy))

    f = interp1d(xPSF, vPSF)
    yPSF = f(newGrid)  # nice big PSF

    [X, Y] = np.mgrid[0:np.max(arcl):dx, zmin:zmax + dz:dz]

    interp2 = RegularGridInterpolator((z, arcl), inmap, bounds_error=False, fill_value=None)  # ((x, y), data)
    inmap_prime = interp2((Y, X))

    xpad = yPSF.shape[0]
    ypad = yPSF.shape[1]
    inmap_padded = padding(inmap_prime, xpad, ypad)

    PS_FFT = fft2(yPSF)
    v_FFT = fft2(inmap_padded)

    PS_FFT = fftshift(PS_FFT)
    v_FFT = fftshift(v_FFT)

    M, N = np.shape(v_FFT)

    mask_radius = mask_perc * M
    mask = create_circular_mask(M, N, radius=mask_radius)
    v_FFT = v_FFT * mask

    H = PS_FFT
    Hdiv = (np.conj(H) / (np.square(np.abs(H)) + 8))
    PHI = v_FFT

    wnrdeconv = Hdiv * PHI 
    sigma = ifftshift(wnrdeconv)
    sigma_fft = ifft2(sigma)
    sigma_plot = ifftshift(sigma_fft)

    r_0, c_0 = np.asarray(inmap_prime.shape)
    r_1, c_1 = np.asarray(sigma_plot.shape)
    outmap = np.transpose(
        10 * np.real(sigma_plot[r_1 // 2 - r_0 // 2:r_1 // 2 + r_0 // 2, c_1 // 2 - c_0 // 2:c_1 // 2 + c_0 // 2]))
    return outmap


# generates a, sigma, and gamma for PSF given a permittivity
def gen_psf(perm, psf_file):
    # inputs: permitivitty and the PSF file in CSV format
    perm10 = round(perm * 10) # multiply permitivitty by ten to make indexing easier, tenth granularity is good enough

    psf_data = np.loadtxt(psf_file, delimiter=',', skiprows=1) # load the PSF file
    # pull out columns:
    e = psf_data[:, 0]*10 # permitivitty
    a = psf_data[:, 1] # alpha
    sigma = psf_data[:, 2]
    gamma = psf_data[:, 3]
    erange = np.arange(0, 300, 1, dtype=int) # make range of 0 to 300 to interpolate over (0 to 30 F/m)
    # interpolate outputs:
    ainterp = np.interp(erange, e, a)
    sinterp = np.interp(erange, e, sigma)
    ginterp = np.interp(erange, e, gamma)
    return ainterp[perm10], sinterp[perm10], ginterp[perm10]
