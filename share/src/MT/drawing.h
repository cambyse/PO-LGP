#include <Core/array.h>

#include <gtk/gtk.h>
//struct cairo_t;

struct Drawing{
  cairo_t *cr;
  GtkWidget *widget;
  int width, height;
  double xl, xh, yl, yh;

  Drawing(cairo_t *_cr, GtkWidget *_widget):cr(_cr),widget(_widget){}

  void draw_function(const arr& f){
    if(f.nd==2){ for(uint i=0;i<f.d0;i++) draw_function(f[i]); return; }
    CHECK(f.nd==1 && f.N>0,"");
    cairo_move_to (cr, X(0), Y(f(0)));
    for(uint i=1; i<f.N;i++) cairo_line_to (cr, X(i), Y(f(i)));
    cairo_set_line_width (cr, 2.0);
    cairo_set_line_cap (cr, CAIRO_LINE_CAP_ROUND);
    cairo_stroke (cr);
  }

  double X(double x){ return (x-xl)/(xh-xl)*width; }
  double Y(double y){ return -(y-yl)/(yh-yl)*height; }

  void setRange(double x_lo, double x_hi, double y_lo, double y_hi, uint margin=20){
    GtkAllocation allo;
    gtk_widget_get_allocation(widget, &allo);
    width=allo.width - 2*margin;
    height=allo.height - 2*margin;
    cairo_translate(cr, margin, margin+height);
    xl=x_lo; xh=x_hi; yl=y_lo; yh=y_hi;
    cairo_set_source_rgb(cr, .7, .7, .7);
    cairo_set_line_width(cr, 1);
    cairo_move_to(cr, 0, Y(0));    cairo_rel_line_to(cr, width, 0);
    cairo_move_to(cr, X(0), 0);    cairo_rel_line_to(cr, 0, -height);
    cairo_stroke(cr);

    cairo_set_source_rgb(cr, .2, .2, .2);
    cairo_set_line_width(cr, 2);
    cairo_rectangle(cr, X(xl), Y(yl), width, -height);
    cairo_stroke(cr);

    cairo_select_font_face(cr, "Mono", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
    cairo_set_font_size(cr, 13);
    cairo_move_to(cr, -10, Y((yl+yh)/2.));
    cairo_rotate(cr, -.5*MT_PI);
    text_center(STRING(yl<<':'<<yh));
    cairo_rotate(cr, .5*MT_PI);

    cairo_move_to(cr, X((xl+xh)/2.), 10);
    text_center(STRING(xl<<':'<<xh));
  }

  void text_center(const char *str){
    cairo_text_extents_t extents;
    cairo_text_extents(cr, str, &extents);
    cairo_rel_move_to(cr, -.5*extents.width, +.5*extents.height);
    cairo_show_text(cr, str);
  }
};

