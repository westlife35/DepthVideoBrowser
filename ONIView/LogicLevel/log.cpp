/*
 * log.cpp
 *
 *  Created on: Apr 13, 2014
 *      Author: shiqingcheng
 */

#include <sys/stat.h>
#include <unistd.h>
#include "LogicLevel/log.h"
#include <stdio.h>


void FailureLog(const char *message,int size);
static char log_dir_name[1000] = "log";
void LogInit(const char* log_path)
{
	google::InitGoogleLogging("Libra");
  if(log_path != NULL)
    strcpy(log_dir_name,log_path);

	if (access(log_dir_name, F_OK) != 0) {
    printf("Directory[%s] is not exist!",log_dir_name);
    mkdir(log_dir_name, 777);
    chmod(log_dir_name, 0777);
  }
	FLAGS_log_dir = log_dir_name;

  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = true;
  FLAGS_stop_logging_if_full_disk = true;
  google::InstallFailureWriter (&FailureLog);
	//google::SetLogDestination(google::ERROR,"log/prefix_");
}

void FailureLog(const char *message,int size)
{
  char file_name[1024];
  sprintf(file_name,"%s/Libra.INFO",log_dir_name);
  FILE* fp = fopen(file_name,"a");
  if (fp) {
    fwrite(message, sizeof(char), size, fp);
  }
  fflush(fp);
  fclose(fp);
}

