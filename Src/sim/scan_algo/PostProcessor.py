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

    ravg = np.mean(r)

    thetastart = 0
    thetaend = 360
    zstart = min(z)
    zend = max(z)

    thetanew = np.linspace(thetastart, thetaend, interpnumx)
    znew = np.linspace(zstart, zend, interpnumy)

    xv, yv = np.meshgrid(thetanew, znew)

    newgrid = griddata((theta, z), v, (xv, yv), method='cubic', fill_value=0)

    return newgrid, ravg, thetastart, thetaend, zstart, zend


# takes unordered x, z, v (columns) data in and arranges into x, z, v array for further processing
def arrange_face(indata, interpnumx=100, interpnumy=100):
    # input data in x, y, v
    # x: use as is
    # y: use as is
    # interpnumx/y: not too critical, but make larger than input array by about 2x.
    x = indata[:, 0]
    y = indata[:, 1]
    v = indata[:, 2]

    xstart = min(x)
    xend = max(x)
    ystart = min(y)
    yend = max(y)

    xnew = np.linspace(xstart, xend, interpnumx)
    ynew = np.linspace(ystart, yend, interpnumy)

    xv, yv = np.meshgrid(xnew, ynew)

    newgrid = griddata((x, y), v, (xv, yv), method='linear', fill_value=0)

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
    bigG = (a / (sig * np.sqrt(2 * np.pi))) * np.exp(-(np.square(xPSF)) / (2 * np.square(sig)))
    L = (g / np.pi) / (np.abs(xPSF) + np.square(g))

    vPSF = np.convolve(bigG, L)
    xPSF = np.linspace(-600, 600, 2 * np.size(xPSF) - 1, endpoint=True)

    dz = (zmax / np.size(inmap[:, 1])) / 4
    dx = (np.max(x) / np.size(x)) / 4
    xx, yy = np.mgrid[-400:400 + dx:dx, -400:400 + dz:dz]  # -400 to 400 hopefully big enough
    # some padding room
    newGrid = np.sqrt(np.square(xx) + np.square(yy))

    f = interp1d(xPSF, vPSF)
    yPSF = f(newGrid)  # nice big PSF

    [X, Y] = np.mgrid[0:np.max(x):dx, zmin:zmax + dz:dz]

    interp2 = RegularGridInterpolator((z, x), inmap, bounds_error=False, fill_value=None)  # ((x, y), data)
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

    wnrdeconv = Hdiv * PHI  # these are sometimes not the same size, how to fix?
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
    perm10 = round(perm * 10)

    psf_data = np.loadtxt(psf_file, delimiter=',', skiprows=1)
    e = psf_data[:, 0]*10
    a = psf_data[:, 1]
    sigma = psf_data[:, 2]
    gamma = psf_data[:, 3]
    erange = np.arange(0, 300, 1, dtype=int)

    ainterp = np.interp(erange, e, a)
    sinterp = np.interp(erange, e, sigma)
    ginterp = np.interp(erange, e, gamma)
    return ainterp[perm10], sinterp[perm10], ginterp[perm10]
