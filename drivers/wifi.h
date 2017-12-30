/*
 * wifi.h
 *
 *  Created on: 30.12.2017
 *      Author: Mateusz
 */

#ifndef WIFI_H_
#define WIFI_H_


void wifi_initialize();

void wifi_send(char *s);
void wifi_receiver_task ();

#endif /* WIFI_H_ */
