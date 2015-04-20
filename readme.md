## IMPORTANT DIRECTORIES

- ${HOME}/too/wsnet
  - contains the wsnet source files
  - It is useful to check internal wsnet functions

- ${HOME}/too/wsnet-module/user_models 
  - contains subdirectories with the code of the user

- ${HOME}/too/wsnet-module/user_models/${dir}
  - ${dir} is the module name
  - ${dir} contains the .c source file


- ${HOME}/too/wsnet-module/lib
  - contains the user's libraries that will be used by wsnet
  - This path is declared in the $WSNET_MODDIR variable

- ${HOME}/sim
  - contains the directory for the simulation

- ${HOME}/bin
  - contains the user's scripts
  - This can be a fork of a github repository (it should be the case) so that we have your script stored somewhere


## NOTES
  1. NOTE: wsnet is already installed and running (at least I hope). You can check by launching: 
  
  ```sh
    wsnet -c /usr/local/wsnet-2.0/demo/cbr.xml
  ```
  
  2. It is better to create a github account. It will ease a lot of things. you can fork the repository [7ahiry](https://github.com/7ahiry) and clone your own repository in order to keep track of your work and in order for other to use your work later.

## TODO
NOTE : if you have forked [7ahiry](https://github.com/7ahiry) then use your own, if not, use : (https://github.com/7ahiry)

  1. Create/clone the ${HOME}/bin. ${HOME}/bin is already in your ${PATH}
  ```sh
  cd ${HOME}
  git clone https://github.com/7ahiry/bin
  ```
  
  2. Clone simulation directory. It contains xml description of the simulation, and a topology configuration.
  ```sh
  cd ${HOME}/sim
  git clone https://github.com/7ahiry/sim_gr
  ```
  
  3. Clone basic modules and compile them. The two following modulesdo not require qny change from you. 
  ```sh
  cd ${HOME}/too/wsnet-module/user_models
  git clone https://github.com/7ahiry/filestatic
  cd filestatic
  wsnet_insert_module.sh mobility filestatic
  ```
  ```sh
  cd ${HOME}/too/wsnet-module/user_models
  git clone https://github.com/7ahiry/myenergy
  cd myenergy
  wsnet_insert_module.sh energy myenergy
  ```
  Note that the wsnet_insert_module.sh is in ${HOME}/bin
  
  4. Clone the grandient routing file skeleton, edit, complete, compile and run simulation
  ```sh
  cd ${HOME}/too/wsnet-module/user_models
  git clone https://github.com/7ahiry/gr
  cd gr
  gvim gr.c
  [modify file]
  wsnet_insert_module.sh application gr 
  cd ${HOME}/sim/sim_gr
  wsnet -c gradient.xml
  ```
  This files contains a working gradient routing. You need to improve it. 
    1. Divide into 3 groups.
      1. Group 1 will focus on the statistics plotting.
      2. Group 2 will focus on the reduction of duplication
      3. Group 3 will focus on energy-aware mechanisms.
  5. Plot statistics. Use the output of your simulation to create statistics such as: delivery ratio, number of duplicate, average number of node involved in the routing process. etc... Based on the work of Group 1, comapre the results of Group 2, Group 3 and original version of gradient.
  6. Follow your code with git, push your code to your repository, pull request to me. 

