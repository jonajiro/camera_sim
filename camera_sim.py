from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import numpy as np
import sys

class PlotObject(gl.GLViewWidget):
    sigUpdate = QtCore.Signal(float, float)
    App = None

    def __init__(self, app=None):

        if self.App is None:
            if app is not None:
                self.App = app
            else:
                self.App = QtGui.QApplication([])
        super(PlotObject,self).__init__()

        self.Gridxy = gl.GLGridItem()
        self.Axes = gl.GLAxisItem()
        self.Gridxy.setSize(100,100,3)
        self.Gridxy.setSpacing(1,1,0)
        self.Gridxy.translate(0, 0, 0)

        self.Poss = []
        self.Cols = []
        self.Sizes = []
        self.GlobalInds = []

        self.Plot = gl.GLScatterPlotItem()

        self.addItem(self.Plot)
        self.addItem(self.Gridxy)

        self.addItem(self.Axes)
        self._downpos = []
        
        ##表示プロットの位置設定#############################
        self.pos_data = np.zeros([60,3])
        for i in range(10):
            self.pos_data[i,0] = i
            self.pos_data[i,1] = 0
        for i in range(10):
            self.pos_data[i+10,0] = i
            self.pos_data[i+10,1] = 9
        for i in range(10):
            self.pos_data[i+20,0] = 0
            self.pos_data[i+20,1] = i
        for i in range(10):
            self.pos_data[i+30,0] = 9
            self.pos_data[i+30,1] = i
        for i in range(10):
            self.pos_data[i+40,0] = 0
            self.pos_data[i+40,2] = i
        for i in range(10):
            self.pos_data[i+50,0] = 9
            self.pos_data[i+50,2] = i
        ##表示プロットの位置設定ここまで######################
        
        self.opts['fov'] = 65#画角設定(焦点距離に影響)

        self.color_data = np.ones([len(self.pos_data),3])
        self.color_data[:,2] = self.color_data[:,2]*0
        self.color_data[:,1] = self.color_data[:,1]*0
        self.setWindowTitle('Pick 3D points')

    def mousePressEvent(self, ev):
        super(PlotObject, self).mousePressEvent(ev)
        self._downpos = self.mousePos

    def mouseReleaseEvent(self, ev):
        super(PlotObject, self).mouseReleaseEvent(ev)
        if self._downpos == ev.pos():
            x = ev.pos().x()
            y = ev.pos().y()
            if ev.button() == 2 :
                self.mPosition()
            elif ev.button() == 1:
                x = x - self.width() / 2
                y = y - self.height() / 2

        self._prev_zoom_pos = None
        self._prev_pan_pos = None
    def keyReleaseEvent(self, ev):
        super(PlotObject, self).keyReleaseEvent(ev)
        if ev.key() == 67:
            focus_len = 0.5*self.width()/np.tan(0.5*self.opts['fov']/180*np.pi)
            self.grabFramebuffer().save("cap" +str(focus_len) + ".png")

    def plotGLPlot(self):
        pos_data = self.pos_data
        color_data = self.color_data
        length = len(color_data)
        colors = [None for k in range(length)]
        for k in range(length):
            rgb_val = np.zeros(3,dtype=float)
            rgb_val[0] = color_data[k,0]
            rgb_val[1] = color_data[k,1]
            rgb_val[2] = color_data[k,2]
            colors[k] = (rgb_val[0],rgb_val[1],rgb_val[2])
            self.Poss.append([pos_data[k,0], pos_data[k,1], pos_data[k,2]])
            self.Cols.append([rgb_val[0], rgb_val[1], rgb_val[2]])
            self.Sizes.append(1)
            self.GlobalInds.append(k)

        poss = np.array([0,0,0])
        cols = np.array([0,0,0,0])
        for i in range(len(self.Poss)):
            possb = np.array([self.Poss[i][0], self.Poss[i][1], self.Poss[i][2]])
            poss = np.vstack([poss,possb])
            colsb = np.array([self.Cols[i][0], self.Cols[i][1], self.Cols[i][2],1])
            cols = np.vstack([cols,colsb])

            
        self.Plot = gl.GLScatterPlotItem()
        self.Plot.setData(pos=poss, size=0.1, color=cols, pxMode=False)
        
        self.addItem(self.Plot)
        self.show()

    def mPosition(self):
        mx = self._downpos.x()
        my = self._downpos.y()
        self.Candidates = []
        self.Dist = []

        view_w = self.width()
        view_h = self.height()

        x = 2.0 * mx / view_w - 1.0
        y = 1.0 - (2.0 * my / view_h)
        z = 1.0

        PM = np.matrix([self.projectionMatrix().data()[0:4],
                           self.projectionMatrix().data()[4:8],
                           self.projectionMatrix().data()[8:12],
                           self.projectionMatrix().data()[12:16]])
        PMi = np.linalg.inv(PM)
        VM = np.matrix([self.viewMatrix().data()[0:4],
                           self.viewMatrix().data()[4:8],
                           self.viewMatrix().data()[8:12],
                           self.viewMatrix().data()[12:16]])

        ray_clip = np.matrix([x, y, -1.0, 1.0]).T
        ray_eye = PMi * ray_clip
        ray_eye[2] = -1
        ray_eye[3] = 0
        ray_world = VM * ray_eye
        ray_world = ray_world[0:3].T
        ray_world = ray_world / np.linalg.norm(ray_world)
        O = np.matrix(self.cameraPosition())
        for i, C in enumerate(self.Poss):
            OC = O - C
            b = np.inner(ray_world, OC)
            b = b.item(0)
            c = np.inner(OC, OC)
            c = c.item(0) - np.square((0.1 / 2 ))
            bsqr = np.square(b)
            if (bsqr - c) >= 0:
                self.Candidates.append(self.GlobalInds[i])
                CO=C - O
                dist = np.linalg.norm(CO - np.inner(ray_world, CO)*ray_world)
                self.Dist.append(abs(dist))

        pick_index = self.Candidates[self.Dist.index(min(self.Dist))]
        possd = np.array([self.Poss[pick_index][0],self.Poss[pick_index][1],self.Poss[pick_index][2]])

        poss = np.array(possd)
        cols = np.array([1,0,0,1])
           
        self.Plot = gl.GLScatterPlotItem()
        self.Plot.setData(pos=poss, size=0.05, color=cols, pxMode=False)

        self.Plotl = gl.GLLinePlotItem()
 
        Line_PLOT = np.zeros([2,3])
        Line_PLOT[0,0] = O[0,0]
        Line_PLOT[0,1] = O[0,1]
        Line_PLOT[0,2] = O[0,2]

        Line_PLOT[1,0] = poss[0]
        Line_PLOT[1,1] = poss[1]
        Line_PLOT[1,2] = poss[2]
 
        self.Plotl.setData(pos=Line_PLOT, width=1, antialias=False)

        self.addItem(self.Plot)
        self.addItem(self.Plotl)
        self.show()

if __name__ == "__main__":
    v = PlotObject()
    v.plotGLPlot()
    v.show()
    
    sys.exit(v.App.exec_())