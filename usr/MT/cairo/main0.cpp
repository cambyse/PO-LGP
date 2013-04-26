#include <MT/array.h>
#include <MT/drawing.h>
#include <MT/gtk.h>
#include <gtk/gtk.h>
//#include <gtk/gtk.h>
//#include <gdk/gdkcairo.h>

static gboolean draw_cb(GtkWidget *widget, cairo_t *cr, gpointer data){   
   cr = gdk_cairo_create( gtk_widget_get_window (widget));

   cairo_set_source_rgb(cr, 1, 1, 1);
   cairo_paint(cr);

   // static byteA img;
   // static cairo_surface_t *image = NULL;
   // if(!img.N) read_ppm(img, "z.ppm");
   // if(!image) image = cairo_image_surface_create_for_data (
   //           img.p, CAIRO_FORMAT_RGB24,
   // 	     img.d1, img.d0, 3*img.d1);
   // cairo_set_source_surface(cr, image, 10, 10);
   // cairo_paint(cr);    

   Drawing drawing(cr, widget);
   drawing.setRange(0, 10, -1, 1);

   arr f;
   f.setGrid(1, 0., 50., 49);
   f = sin(f);
   f.reshape(5,10);
   drawing.draw_function(f);

   cairo_destroy(cr);

   return FALSE;
}

int main (int argc, char *argv[]){
   gtk_init (&argc, &argv);

   GtkWidget *window;
   GtkWidget *da;

   window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
   g_signal_connect (window, "destroy", G_CALLBACK (gtk_main_quit), NULL);

   da = gtk_drawing_area_new();
   gtk_widget_set_size_request (da, 300, 300);
   g_signal_connect (da, "expose-event", G_CALLBACK(draw_cb),  NULL);

   gtk_container_add (GTK_CONTAINER (window), da);
   gtk_widget_show(da);
   gtk_widget_show (window);

   gtk_main ();
   return 0;
}
