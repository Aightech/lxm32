# lxm32

## Information
- **Brief**: Library to control a Lexium32A driver from Schneider. 
- **Languages**: C++
- **Libraries**: 
- **Note**: /
- **Compatibility**:

| Ubuntu           | Window10         | MacOS            |
|:----------------:|:----------------:|:----------------:|
|:x:|:heavy_check_mark:|:grey_question:   |


## Building
### Ubuntu
#### Steps
- Clone the repository and go inside.
```bash
git clone https://gitlab-dev.isir.upmc.fr/devillard/lxm32.git && cd lxm32
```
- Create a build directory and go inside.
- Configure the project.
- Build the project.
```bash
mkdir build && cd build && cmake .. && cmake --build .
```