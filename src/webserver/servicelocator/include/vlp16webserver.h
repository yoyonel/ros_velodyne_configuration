#ifndef __VLP16WEBSERVER_H__
#define __VLP16WEBSERVER_H__


#include "velodynewebserver_meta.h" // for: VelodyneWebServerMeta<eCmd>
#include <enum.h>   // for: BETTER_ENUM
#include <stdint.h> // for uint8_t


// On definit la liste des requetes (webserver) gerees par le VLP16
BETTER_ENUM( eVLP16WebServerRequests, uint8_t,
             settings=1,
             status,
             info,
             diag
             );

// On utilise cette liste de requetes
// pour instancier un WebServer (VelodyneWebServerMeta) pour le VLP16
typedef VelodyneWebServerMeta<eVLP16WebServerRequests> VLP16WebServer;

#endif // __VLP16WEBSERVER_H__
