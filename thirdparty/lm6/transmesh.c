

/*----------------------------------------------------------*/
/*															*/
/*						TRANSMESH V 4.0						*/
/*															*/
/*----------------------------------------------------------*/
/*															*/
/*	Description:		convert mesh file from/to ascii/bin	*/
/*	Author:				Loic MARECHAL						*/
/*	Creation date:		mar 08 2004							*/
/*	Last modification:	feb 27 2014							*/
/*															*/
/*----------------------------------------------------------*/


/*----------------------------------------------------------*/
/* Includes													*/
/*----------------------------------------------------------*/

#define TRANSMESH 1

#include <libmesh6.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


/*----------------------------------------------------------*/
/* Check args and copy each fields from infile to outfile	*/
/*----------------------------------------------------------*/

int main(int argc, char **argv)
{
	int i, j, NmbTyp, SolSiz, TypTab[ GmfMaxTyp ], InpIdx, OutIdx, FilVer=0, InpVer, OutVer=1, dim;
	long NmbLin;
//	float f;
//	double d;
	char *InpNam, *OutNam, *VerNam;

	switch(argc)
	{
		case 1 :
		{
			printf("\nTRANSMESH v4.0, feb 27 2014, Loic MARECHAL / INRIA\n");
			printf(" Usage : transmesh source_name destination_name (-options)\n");
			printf(" optional arguments : -v output_file_version\n");
			printf(" version 1: 32 bits integers, 32 bits reals, file size < 2 GigaBytes\n");
			printf(" version 2: 32 bits integers, 64 bits reals, file size < 2 GigaBytes\n");
			printf(" version 3: 32 bits integers, 64 bits reals, file size < 8 ExaBytes\n");
			printf(" version 4: 64 bits integers, 64 bits reals, file size < 8 ExaBytes\n");
			exit(1);
		}break;

		case 3 :
		{
			InpNam = *++argv;
			OutNam = *++argv;
		}break;

		case 5 :
		{
			InpNam = *++argv;
			OutNam = *++argv;
			VerNam = *++argv;

			if(!strcmp(VerNam, "-v"))
			{
				FilVer = atoi(*++argv);

				if( (FilVer < 1) || (FilVer > 4) )
				{
					printf("Wrong size type : %d\n", FilVer);
					exit(1);
				}
			}
		}break;

		default :
		{
			printf("Wrong number of arguments.\n");
			exit(1);
		}break;
	}

	if(!(InpIdx = GmfOpenMesh(InpNam, GmfRead, &InpVer, &dim)))
	{
		fprintf(stderr,"Source of error : TRANSMESH / OPEN_MESH\n");
		fprintf(stderr,"Cannot open %s\n", InpNam);
		return(1);
	}

	if(FilVer)
		OutVer = FilVer;
	else
		OutVer = InpVer;

	if(!(OutIdx = GmfOpenMesh(OutNam, GmfWrite, OutVer, dim)))
	{
		fprintf(stderr,"Source of error : TRANSMESH / OPEN_MESH\n");
		fprintf(stderr,"Cannot open %s\n", OutNam);
		return(1);
	}

	for(i=0;i<=GmfMaxKwd;i++)
	{
		if(!strcmp(GmfKwdFmt[i][0], "Reserved") || !strcmp(GmfKwdFmt[i][0], "End"))
			continue;

		GmfGotoKwd(InpIdx, i);

		if(strcmp("i", GmfKwdFmt[i][2]))
		{
			if((NmbLin = GmfStatKwd(InpIdx, i)))
				GmfSetKwd(OutIdx, i);
			else
				continue;
		}
		else if((strcmp("sr", GmfKwdFmt[i][3])))
		{
			if((NmbLin = GmfStatKwd(InpIdx, i)))
				GmfSetKwd(OutIdx, i, NmbLin);
			else
				continue;
		}
		else
		{
			if((NmbLin = GmfStatKwd(InpIdx, i, &NmbTyp, &SolSiz, TypTab)))
				GmfSetKwd(OutIdx, i,  NmbLin, NmbTyp, TypTab);
			else
				continue;
		}

		printf("Parsing %s : %ld item\n", GmfKwdFmt[i][0], NmbLin);

		for(j=1;j<=NmbLin;j++)
			GmfCpyLin(InpIdx, OutIdx, i);
	}

	if(!GmfCloseMesh(InpIdx))
		return(1);

	if(!GmfCloseMesh(OutIdx))
		return(1);

	return(0);
}
