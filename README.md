# AudioClassification

## Installation:

    git clone https://github.com/hoergems/AudioClassification.git
    cd AudioClassification && mkdir build && cd build
    cmake -DCMAKE_INSTALL_PREFIX=<install folder>
    make && make install

## Usage
Open a terminal and run

    source <install folder>/share/oppt/setup.sh

or add this line to your .bashrc file. \<install folder\> is the installation folder specified in the cmake command above. In the same terminal you can then run the problem with the provided config file, e.g.

    cd <oppt folder>/bin
    ./abt --cfg <folder where this repo is cloned into>/cfg/AudioClassification.cfg
