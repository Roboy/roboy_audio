/* 
 * File:   plot.h
 * Author: letrend
 *
 * Created on April 26, 2015, 9:58 PM
 */

#ifndef PLOT_H
#define	PLOT_H

#include <string>
#include "gnuplot_i.hpp" //Gnuplot class handles POSIX-Pipe-communikation with Gnuplot

static class Plot{
public:
    Plot(){
        N = 5;
        figure.resize(N);
        for (uint n=0;n<N;n++) figure.at(n) = new Gnuplot;
    }
    std::vector<Gnuplot*> figure;
    uint N;
    void array(float* array, uint N, std::string name, uint fig){
        std::vector<float> x,y;
        x.resize(N); y.resize(N); 
        for (uint i=0;i<N;i++)  {
            x.at(i)=i;
            y.at(i)=array[i];
        }
        figure.at(fig)->set_style("lines").plot_xy(x,y,name);
    }

    void array(uint* array, uint N, std::string name, uint fig){
        std::vector<uint> x,y;
        x.resize(N); y.resize(N); 
        for (uint i=0;i<N;i++)  {
            x.at(i)=i;
            y.at(i)=array[i];
        }
        figure.at(fig)->set_style("lines").plot_xy(x,y,name);
    }
    
    void array(uint16_t* array, uint N, std::string name, uint fig){
        std::vector<uint> x,y;
        x.resize(N); y.resize(N); 
        for (uint i=0;i<N;i++)  {
            x.at(i)=i;
            y.at(i)=array[i];
        }
        figure.at(fig)->set_style("lines").plot_xy(x,y,name);
    }
    void array(int16_t* array, uint N, std::string name, uint fig){
        std::vector<int> y1,y2;
        y1.resize(N/2); y2.resize(N/2); 
        uint j=0;
        for (uint i=0;i<N/2;i+=2)  {
            y1.at(j)=array[i];
            y2.at(j)=array[i+1];
            j++;
        }
        figure.at(fig)->set_style("lines").plot_x(y1,name);
        figure.at(fig)->set_style("lines").plot_x(y2,name);
    }
    template <typename T>
    void array(std::vector<T> array, std::string name, uint fig){
        figure.at(fig)->set_style("lines").plot_x(array,name);
    }
    
    template <typename T1, typename T2>
    void array(T1* array1, T2* array2, uint N, std::string name, uint fig){
        std::vector<float> x,y;
        x.resize(N); y.resize(N); 
        for (uint i=0;i<N;i++)  {
            x.at(i)=array1[i];
            y.at(i)=array2[i];
        }
        figure.at(fig)->set_style("lines").plot_xy(x,y,name);
    }

    void clear(uint fig){
        figure.at(fig)->reset_plot();
        if (counter%1000==0)
            figure.at(fig)->remove_tmpfiles();
        counter++;
        
    }
    
    void clearAll(){
        for (uint n=0;n<N;n++){ 
            delete figure.at(n);
            figure.at(n) = new Gnuplot;
        }
    }
    void setYaxis(uint fig, float min, float max){
        figure[fig]->set_yrange(min, max);
    }
    void set_ylogscale(uint fig){
        figure[fig]->set_ylogscale();
    }
    uint counter = 1;
}plot;
#endif	/* PLOT_H */
