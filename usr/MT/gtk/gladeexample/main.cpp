#include <gtk/gtk.h>

extern "C" void on_value_changed(GtkObject *object, void *user_data){
  printf("hallob data=%i\n", *(int*)user_data);
}

extern "C" void button_click(GtkObject *object, void *user_data){
  printf("hallob data=%i\n", *(int*)user_data);
}

extern "C" void on_window_destroy (GtkObject *object, void *user_data){
  printf("exit\n");
  gtk_main_quit();
}

int main (int argc, char *argv[]){
        GtkBuilder              *builder;
        GtkWidget               *window;
        
        gtk_init (&argc, &argv);
        
        builder = gtk_builder_new ();
        gtk_builder_add_from_file (builder, "test.glade", NULL);

        window = GTK_WIDGET (gtk_builder_get_object (builder, "window"));
        gtk_builder_connect_signals (builder, NULL);          
        g_object_unref (G_OBJECT (builder));
        
        gtk_widget_show (window);       
        gtk_main ();
        
        return 0;
}
