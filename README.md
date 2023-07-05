## OptI-ProMP


This repository provides a multi-objective optimization framework satisfying system and environment constraints. It leverages on a machine learning-based technique to initialize the optimization problem. The initialization stems from the generation of the well-known probabbilistic movement primitives as a initial optimization solution learned from data collected from human expert for the desired task. The OptI-ProMP framework is initially proposed for moving in 3d-space. It is modular in the sense that optimisation objectives can be reduced to 2d environments (where motion is constrained in one direction, as reaching for object on a fridge shelf). OptI-ProMP is tested on 3-dof scara arm and 7-dof franka arm.

The scientific content of this work can be found @https://ieeexplore.ieee.org/abstract/document/9926518. To cite this work, please refer to the proceedings of the International Conference on Automation Science and Engineering (CASE 2022), with the following citation:

```
@inproceedings{mghames2022environment,
  title={Environment-aware Interactive Movement Primitives for Object Reaching in Clutter},
  author={Mghames, Sariah and Hanheide, Marc},
  booktitle={2022 IEEE 18th International Conference on Automation Science and Engineering (CASE)},
  pages={493--498},
  year={2022},
  organization={IEEE}
}
```

This repo has dependency on panda_moveit_config and moveit_tutorials packages, please add them to this repo after cloning.

```
We welcome any issue or collaboration. You can reach out @sariahmghames@gmail.com
```

