/**
 * DisplayControl.hpp
 *
 * This file contains the declaration of the DisplayControl class.
 * DisplayControl is the class that handles the organization of the
 * LittleV Graphics Library (LVGL) objects on the screen of the brain.
 */
#pragma once // makes sure the file is only included once
#include "main.h" // gives access to objects not declared here (LVGL objects)
class DisplayControl
{

    /* --------------------- Tabview Elements -------------------- */
    static lv_obj_t * mtabview; // contains the whole tabview

    static lv_obj_t * mtabview_odom; // tabview page with odom debugger
    OdomDebug modom; // odom debugger

    static lv_obj_t * mtabview_auton; // tab for setting auton
    static lv_obj_t * mtabview_auton_dropdown; // autons to choose from
    static lv_res_t tabview_auton_dropdown_action(lv_obj_t * idropdown); // event handler

    static lv_obj_t * mtabview_graph; // tabview page with graph
    static lv_obj_t * mtabview_graph_chart; // graph
    static lv_chart_series_t * mtabview_graph_chart_series_0; // chart series...
    static lv_chart_series_t * mtabview_graph_chart_series_1;
    static lv_chart_series_t * mtabview_graph_chart_series_2;
    static lv_chart_series_t * mtabview_graph_chart_series_3;
    static lv_chart_series_t * mtabview_graph_chart_series_4;
    static lv_chart_series_t * mtabview_graph_chart_series_5;
    static lv_chart_series_t * mtabview_graph_chart_series_6;

    static lv_obj_t * mtabview_misc; // extra tabview page for anything
    static lv_obj_t * mtabview_misc_container; // container on the misc page to hold elements
    static lv_obj_t * mtabview_misc_label; // text box on misc page
    static lv_obj_t * mtabview_misc_label_2; // second text box on misc page

    /* -------------------------- Styles ------------------------- */
    static lv_style_t mstyle_tabview_indic; // for page indicator line
    static lv_style_t mstyle_tabview_btn; // for page header
    static lv_style_t mstyle_tabview_btn_tgl; // for selected page header
    static lv_style_t mstyle_tabview_btn_pr; // for pressed page header
    static lv_style_t mstyle_tabview_container; // for page background
    static lv_style_t mstyle_text; // for text

  public:
    DisplayControl(); // constructor that sets everything up, like styles and positioning

    void setOdomData(); // updates the values for OdomDebug
    void setAutonDropdown(); // updates the dropdown to match sd card at the start of the program,
                             // to ensure the auton set on the sd card is always the same as the
                             // auton displayed on the dropdown.

    void setChartData(int iseries,
                      double ivalue); // sets the value of one of the series in the chart

    void setMiscData(int ilabel,
                     std::string itext); // sets the information displayed in the misc tab
};

namespace def
{
extern DisplayControl
    display; // declares the display object as extern, to make sure it only gets constructed once
}