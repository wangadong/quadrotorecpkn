/*
 ============================================================================
 Name        : hello.c
 Author      : 
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
static int num=0;
void ch(int k,int n){
	int i;
	if(k<=n){
		if(k>n-10)
			num++;
		k+=4;
		for(i=0;i<7;i++){
		ch(k,n);
		k++;
		}
	}
}
int main(void){
	int k=0;
	ch(k,90);
	printf("%d",num*2);
	return EXIT_SUCCESS;
}
