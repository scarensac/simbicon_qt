#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

class ConsoleWindow;
class SimulationWindow;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

private:
    Ui::MainWindow *ui;

public:
    explicit MainWindow(int argc, char *argv[], QWidget *parent = 0);
    ~MainWindow();    

protected:
    SimulationWindow* sim_window;
    ConsoleWindow* console_window;

    void link_signals();

    void closeEvent(QCloseEvent *event);

    void run_optimisation();

public slots:
    void mode_changed(int);
    void select_path_fusing_controlers_clicked();
    void start_click();

protected slots:
    void end_simulation();
};
//*
#include <shark/ObjectiveFunctions/AbstractObjectiveFunction.h>
#include <boost/scoped_ptr.hpp>
double cma_program(std::string save_folder_name, std::string solution_folder , int optimisation_type, int nb_iter_max=60, int cma_number=-1);

class SimBiConFramework;

class SimbiconOnjectiveFunction : public shark::SingleObjectiveFunction
{
public:

    SimbiconOnjectiveFunction();

    void proposeStartingPoint(SearchPointType & startingPoint)const {
        //and we set the starting point
        startingPoint=variable_vector;
    }

    std::string name() const { return "SimbiconOnjectiveFunction"; }

    std::size_t numberOfVariables()const{
        return m_dimensions;
    }
    bool hasScalableDimensionality()const{
        return true;
    }
    void setNumberOfVariables(std::size_t numberOfVariables){
        m_dimensions = numberOfVariables;
    }

    ResultType eval(const SearchPointType & input, int offspring_id=-1);

    template<typename T>
    void write_point_to_structure(SimBiConFramework* con,const T& input);

private:
    std::size_t m_dimensions;

public:
    //I'll store the concerned straj name in a vector for an easier use
    std::vector<std::string> vect_traj_name;

    //just nothing realy
    char* inputFile;

    //this contain the starting point
    shark::RealVector variable_vector;

    //this is the line I execute with system
    std::string exe_line;
};




class SimbiconOnjectiveFunctionGains : public shark::SingleObjectiveFunction
{
public:

    SimbiconOnjectiveFunctionGains(bool use_symetrics_gains, int cma_number,
                                   std::string solution_folder);

    void proposeStartingPoint(SearchPointType & startingPoint)const {
        //and we set the starting point
        startingPoint=variable_vector;
    }

    std::string name() const { return "SimbiconOnjectiveFunction"; }

    std::size_t numberOfVariables()const{
        return m_dimensions;
    }
    bool hasScalableDimensionality()const{
        return true;
    }
    void setNumberOfVariables(std::size_t numberOfVariables){
        m_dimensions = numberOfVariables;
    }

    ResultType eval (const SearchPointType & input, int offspring_id=-1);


private:
    std::size_t m_dimensions;

public:
    //I need them because I'll norlamise the values so that the cma step do smth ...
    std::vector<double> original_val;

    //just nothing realy
    char* inputFile;

    //this contain the starting point
    shark::RealVector variable_vector;

    //other members
    bool use_symetry;
    bool asymetrics_legs;

};

//*/


#endif // MAINWINDOW_H
