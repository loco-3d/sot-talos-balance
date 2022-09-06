# sot-talos-balance

[![Pipeline status](https://gitlab.laas.fr/stack-of-tasks/sot-talos-balance/badges/master/pipeline.svg)](https://gitlab.laas.fr/stack-of-tasks/sot-talos-balance/commits/master)
[![Coverage report](https://gitlab.laas.fr/stack-of-tasks/sot-talos-balance/badges/master/coverage.svg?job=doc-coverage)](https://gepettoweb.laas.fr/doc/stack-of-tasks/sot-talos-balance/master/coverage/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/stack-of-tasks/sot-talos-balance/master.svg)](https://results.pre-commit.ci/latest/github/stack-of-tasks/sot-talos-balance)


Coordination project for the control of the balance of Talos.

## Installation procedure
Assuming you have all the dependencies correctly installed inside `/opt/openrobots`,
the package is installed as follows.
The detailed explanation of the commands is available in the documentation.

```
git clone --recursive git@github.com:loco-3d/sot-talos-balance.git
cd sot-talos-balance
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/openrobots ..
make -j4
make install
