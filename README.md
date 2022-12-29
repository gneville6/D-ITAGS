# Graphically Recursive Simultaneous Task Allocation, Planning, Scheduling, and Execution

## Description

This repository contains the various components of GRSTAPS+E. These include a refactored version of **Forward Chaining
Partial-Order Planning (FCPOP)**; **Incremental Task Allocation Graph Search (Itags)**; and **Graphically Recursive Task
Allocation, Planning, and Scheduling (GRSTAPS)**. Additionally, it contains the original implementations of ...

## Dependencies

### apt

- libeigen3-dev
- libgeos-dev
- libompl-dev

### pip

todo

### mirrored public repositories

- [json](https://github.com/amessing/json) ([original](https://github.com/nlohmann/json))
- [googletest](https://github.com/amessing/googletest) ([original](https://github.com/google/googletest))
- [fmt](https://github.com/amessing/fmt) ([original](https://github.com/fmtlib/fmt))
- [spdlog](https://github.com/amessing/spdlog) ([original](https://github.com/gabime/spdlog))
- [z3](https://github.com/amessing/z3) ([original](https://github.com/Z3Prover/z3))
- [robin-hood-hashing](https://github.com/amessing/robin-hood-hashing) ([original](https://github.com/martinus/robin-hood-hashing))
- [magic enum](https://github.com/amessing/magic_enum) ([original](https://github.com/Neargye/magic_enum))
- [benchmark](https://github.com/amessing/benchmark) ([original](https://github.com/google/benchmark))
- [fiboheap](extern/fiboheap) ([original](https://github.com/beniz/fiboheap))

### other

#### Gurobi

We use [Gurobi](https://www.gurobi.com/) to solve Mixed Integer Linear Programming problems. In order to use this with
the docker, you will need to get a Web
License ([click here](https://www.gurobi.com/academia/academic-program-and-licenses/)) and then put the
associated ```gurobi.lic``` file in ```docker/gurobi```. The docker-compose will mount that file to the docker container
so that it can use gurobi. The license file is in the .gitignore and should under ___NO___ circumstance become part of
the repository (simply do not change that line and this shouldn't ever be something to worry about). You are responsible
for your own gurobi license.

## Usage

This library is used to run experiments for academic papers. Instructions to run the experiments for those papers are
listed below:

### ICRA 2022

TODO

## Development

If you are interested in working on the development of this project please to our wiki for information about setting up
your coding environment, our coding standards, and workflow.

# Citations

### [Forward Chaining Hierarchical Partial-Order Planning](http://robotics.cs.rutgers.edu/wafr2020/wp-content/uploads/sites/7/2020/05/WAFR_2020_FV_43.pdf)

```
Messing, Andrew, and Seth Hutchinson. "Forward Chaining Hierarchical Partial-Order Planning." 
Algorithmic Foundations of Robotics XIV. Springer International Publishing, 2021.
```

### [Incremental Task Allocation Graph Search]()

```
todo
```

### [Graphically Recursive Simultaneous Task Allocation, Planning, and Scheduling]()

```
todo
```

# Licensing

See [LICENSE](LICENSE)# D-ITAGS
# D-ITAGS
# D-ITAGS
