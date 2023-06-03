# sciplot库使用方法记录

## 基本使用
```
    Vec x = linspace(0.0, 5.0, 200);
    Plot2D plot0;
    plot0.drawCurve(x, std::sin(x)).label("sin(x)");
    Plot2D plot1;
    plot1.drawCurve(x, std::cos(x)).label("cos(x)");
    // Use the previous plots as sub-figures in a larger figure. Note that
    // plot0 and plot1 will be deep-copied into fig
    Figure fig = {{plot0, plot1}};
    Canvas canvas = {{fig}};
    Canvas canvas1 = {{fig}};
    canvas.size(750, 750);
    canvas.title("dasdas");
    canvas.show();
    canvas1.show();
```
其中，Vec的定义为：
```
using Vec = std::valarray<double>;
```