# frc-2019
Robot code for FRC 2019 Destination: Deep Space

### Prerequisites

You will need to have Git installed on your computer to, and some sort of code editor. If you don't have a Java code editor installed already, it is recommended you install VS Code, and install the Java Extension Pack.

### Installation

Open a terminal shell (Powershell, Bash, etc.) and use the change directory command, `cd`, to navigate to a folder you want to place robotics projects in. For example, if you want to put projects in "C:\Users\user\robotics", then type you would type the command `cd "C:\Users\user\robotics"`. Note that whatever folder you want to use must exit, if it doesn't you need to create it.
Next, run the command `git clone --recurse-submodules https://github.com/Pigmice2733/frc-2019.git`. This will download this project into a subfolder called "frc-2019".

### Builing and running

In a terminal, navigate to the "frc-2019" folder. To build the code, run "./gradlew build". This may take a minute or two, subsequent builds will be faster. To 
run all unit tests, run "./gradlew test". To deploy code onto an FRC robot, run "./gradlew deploy" - this will require you to be connected to the robot via Ethernet or WiFi.
