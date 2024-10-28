# Experiments 

## Note on ABC version.
The version of ABCBMC we use:
```
commitHash =  5a9a902044182bac824a98cfdccbe9aae411a7c6
Date:   Wed May 17 10:34:14 2023 -0700
```
A binary file we used is also provided within the zip.

As far as we know, there's no update on BMC/PDR recently. And the performance should be approximate.

## How to run
The cluster in which we ran the experiment supports the PBS system to run experiments.
A sample PBS script is provided in this directory. 

The options we used to compile caramel:

| OPTION  | Meaning                          |
| ------- | -------------------------------- |
| INTER   | enable $Intersection$ or $Local$ |
| ROTATE  | enable $Rotation$                |
| PP_CNT  | upper bound of $iLimit$.         |
| PP_TIME | $restartTimeLimit$               |
| MAX_NI  | starting  $iLimit$               |

The options we used to run:

| Tool    | Options                                                      |
| ------- | ------------------------------------------------------------ |
| caramel | -b -e                                                        |
| BAC     | -i ${iLimit} -b -e  -rotation -loop 1500                     |
| BICAR   | -b -e -i ${iLimit} -rotation -bmc 30                         |
| KCAR    | -b -e -i ${iLimit} -rotation -unroll 5                       |
| ABC-BMC | -c "bmc -v;write_cex -a './bmc_res/${name}.cex'"             |
| ABC-PDR | -c "pdr;write_cex -a './pdr_res/${name}.cex'"                |
| AVY     | --reset-cover=1 -a --kstep=2 --shallow-push=1 --tr0=1 --min-suffix=1 --glucose --glucose-inc-mode=0 --min-core=1 --glucose_itp=1 --stick-error=1 --sat-simp=1 --cex |
| navy    | --reset-cover=1 --opt-bmc --kstep=1 --shallow-push=1 --min-suffix=1 --glucose --glucose-inc-mode=0 --sat-simp=1 --glucose_itp=1 ${file_name_list[$subtask]} --cex |

## How to process results

A script is provided in `../graphs/scripts/deal_result.py`.

As to cluster's log of BMC: use regex to filter out the time marks.

The results are provided in the excel file `dataTable.xlsx`

## Drawing the figures
The excel to draw the figures are provided :`dataTable.xlsx`
The scripts to draw scatter plots are provided in `../graphs/scripts/`
