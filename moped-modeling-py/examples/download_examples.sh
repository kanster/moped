#!/bin/bash

wget http://personalrobotics.ri.cmu.edu/moped/moped2_model_examples.tar.gz
tar -xzf moped2_model_examples.tar.gz

mkdir rice_tuscan
mv rice_tuscan.tar.gz rice_tuscan
cd rice_tuscan
tar -xzf rice_tuscan.tar.gz
rm rice_tuscan.tar.gz
cd ..

mkdir fuze_bottle
mv fuze_bottle.tar.gz fuze_bottle
cd fuze_bottle
tar -xzf fuze_bottle.tar.gz
rm fuze_bottle.tar.gz
cd ..

