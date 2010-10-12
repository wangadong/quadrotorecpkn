/*
 ============================================================================
 Name        : cc.c
 Author      : 
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
//realisation
int calcule_family_direct(int n)
{
	if (n<=3)
	{
		int k=2;
		return k;
	}
	if(n>=4&&n<=9)
	{
		int k=2;
		int i=0;
		for(i=0;i<=n-4;i++)
		{
			k+=calcule_family_direct(i);
		}
		return k;
	}
	if(n>=10)
	{
		int k=0;
		int j=0;
		for(j=0;j<=n-4;j++)
		{
			k+=calcule_family_direct(j);
		}
		return k;
	}
}

void main()
{
	int n,k;
	scanf("%d",&n);
	k=calcule_family_direct(n);
	printf("%d",k);
}
