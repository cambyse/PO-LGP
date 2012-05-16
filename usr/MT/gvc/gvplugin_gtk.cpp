#include <graphviz/gvplugin_device.h>

#include <gtk/gtk.h>
#include <cairo.h>

#include "callbacks.h"


static void gtk_initialize(GVJ_t *firstjob) {
  //gtk_init(NULL, NULL);
  GVJ_t *job;
  printf("here\n");
  
  for(job = firstjob; job; job = job->next_active) {
    init(job);
  }
}

static void gtk_finalize(GVJ_t *firstjob) {
  GVJ_t *job;
  printf("there\n");
  
  //gtk_main();
}

static gvdevice_features_t device_features_gtk = {
  GVDEVICE_DOES_TRUECOLOR
  | GVDEVICE_EVENTS,      /* flags */
  {0.,0.},                    /* default margin - points */
  {0.,0.},                    /* default page width, height - points */
  {96.,96.},                  /* dpi */
};

static gvdevice_engine_t device_engine_gtk = {
  gtk_initialize,
  NULL,     /* gtk_format */
  gtk_finalize,
};

gvplugin_installed_t gvdevice_types_gtk[] = {
  {0, "gtk:cairo", 0, &device_engine_gtk, &device_features_gtk},
  {0, NULL, 0, NULL, NULL}
};

static gvplugin_api_t apis[] = {
  {API_device, gvdevice_types_gtk},
  {(api_t)0, 0},
};

gvplugin_library_t gvplugin_gtk_LTX_library = { "gtk", apis };
