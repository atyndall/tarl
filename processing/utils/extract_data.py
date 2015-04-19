import re, csv, sys

fp = open(sys.argv[1], 'r')
msgs = []
cur = ''

for line in fp:
	if line.strip('\r\n= ') == 'Evaluation result':
		msgs.append(cur)
		cur = ''
	
	cur += line
fp.close()

num = []
nom = []

for i, msg in enumerate(msgs[1:]):
	scheme_re = re.search('Scheme:\s+(.*) : (.*)', msg)
	relation_re = re.search('Relation:\s+(.*)', msg)
	rmse_re = re.search('Root mean squared error\s+([\d.-]+)', msg)
	correct_re = re.search('Correctly Classified Instances\s+(?:[\d.-]+)\s+([\d.-]+)', msg)
	fscore_re = re.search('Weighted Avg\.\s+(?:[\d.-]+)\s+(?:[\d.-]+)\s+(?:[\d.-]+)\s+(?:[\d.-]+)\s+([\d.-]+)\s+(?:[\d.-]+)\s+(?:[\d.-]+)\s+(?:[\d.-]+)', msg)
	r2_re = re.search('Correlation coefficient\s+([\d.-]+)', msg)

	print('**')
	print(i)
	print(scheme_re)
	print(relation_re)
	print(rmse_re)
	print(correct_re)
	print(fscore_re)
	print(r2_re)


	if not r2_re:
		tup = (scheme_re.group(1), scheme_re.group(2), relation_re.group(1), rmse_re.group(1), correct_re.group(1), fscore_re.group(1))
		nom.append(tup)
	else:
		tup = (scheme_re.group(1), scheme_re.group(2), relation_re.group(1), rmse_re.group(1), '', '', r2_re.group(1))
		num.append(tup)

	
csvfp = open(sys.argv[2], 'wb')
csvwriter = csv.writer(csvfp)

csvwriter.writerow(('Desc','Classifier','Relation','RMSE','Percent Correct','F-Score','Correlation coeff'))
for n in nom:
	csvwriter.writerow(n)

for n in num:
	csvwriter.writerow(n)

csvfp.close()