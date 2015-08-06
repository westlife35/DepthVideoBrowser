ONIBrowser
===

Usage:
---
    there is a graphical interface for user. It is easy to use it.(Specially, when you browse the oni file in fullscreen mode, you should press "Esc" to go back to the normal interface.) 
      
Design:
---
    There are three levels:Show Level, Logic Level,Cam Level. The files of Logic Level and Cam Level are put in the subfolders of project main folder named by the level's name. The files of Show Level are put in the project main folder
       
**Cam Level**:
    Realize the openning of Cam or the reading of oni file through OpenNI.(three classes with inheriting, make it easy to change to other cam of different patterns or brands)
    
**Logic level**:
    Package the class in Cam Level in class ONICapture, leaving the interfaces for Show Level. The repackaged class is easy for Show Level to use.(only a .cpp file and a .h file)
    
**Show level**:
    Through the using of Qt and the interfaces of Logic Level to get the data of depth and color, show the data in the graphical interface.
    
    
Functions:
---
> * 1.Open the cam. You can view the depth frame and color frame simutaniously. Also, people will be tracked in real time and you can view in the fillscreen mode.


> * 2.Open oni file. You can view the depth frame and color frame simutaniously. Also, people will be tracked in real time and you can view in the fillscreen mode.


> * 3.When browsering the oni file, the oni file will be played circularly and you can paused the playing any time you want.

> * 4.You can change the playing speed in a big range from 0.01 to 100. Notely, the too low or too high speed will not take effect because there are some limits.

> * 5.When playing the oni file, the playing process will be displyed in the right corner of the depth view window.

> * 6.You can change the playing process through dragging the horizontal slider in the bottom of the depth frame view frame when the oni file is played or paused.

> * 7.You can paused the playing and then your mouse cursor will become a cross. You can add a new tracked goal with a new ID or change a current tracked ID to a ID defined by yourself. You can also paused the playing and drag the horizontal slider to make the oni file to show a previous frame to make you can do some change on the previous frame. Notely, the tracking state will be record and a backward will reset the tracking state too.
