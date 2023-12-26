#include <stdio.h>
#include <curl/curl.h>
 
int main(void)
{
  CURL *curl;
  CURLcode res;
 
  curl = curl_easy_init();
    int rowCount, headerCount, incrementSize;

    // Get inputs
    printf("Enter Row Count: ");
    scanf("%d", &rowCount);

    printf("Enter Header Count: ");
    scanf("%d", &headerCount);

    printf("Enter Increment Size: ");
    scanf("%d", &incrementSize);


  if(curl) {
    curl_easy_setopt(curl, CURLOPT_URL, ("192.168.162.125:80/lidar?lidar_scan=%s&lidar_increment=%s&lidar_increment_size=%s", rowCount,headerCount,incrementSize));
    /* example.com is redirected, so we tell libcurl to follow redirection */ 
    /* Perform the request, res will get the return code */
    res = curl_easy_perform(curl);
    /* Check for errors */
    if(res != CURLE_OK)
      fprintf(stderr, "curl_easy_perform() failed: %s\n",
              curl_easy_strerror(res));
 
    /* always cleanup */
    curl_easy_cleanup(curl);
  }
  return 0;
}
