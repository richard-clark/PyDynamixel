from distutils.core import setup

def get_long_description():
    f = open('README.rst')
    data = f.read()
    f.close()
    return data

setup(name = 'pydynamixel',
      packages = ['pydynamixel'],
      version = '1.0.0',
      description = 'A package for controlling Dynamixel servos',
      long_description = get_long_description(),
      author = 'Richard Clark',
      author_email = 'pydev@richard-h-clark.com',
      url = 'http://richard-h-clark.com/pydynamixel',
      keywords = ['dynamixel', 'servo'],
      classifiers = ['Programming Language :: Python',
                     'License :: OSI Approved :: GNU Library or Lesser General Public License (LGPL)',
                     'Operating System :: OS Independent',
                     'Development Status :: 4 - Beta',
                     'Intended Audience :: Developers',
                     'Intended Audience :: Science/Research',
                     'Topic :: System :: Hardware'
                     ]
      )