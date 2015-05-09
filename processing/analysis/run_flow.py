# Note: You need to use Jython to run this script
# Run on a Mac using;
# export CLASSPATH=$CLASSPATH:/Applications/Weka.app/Contents/Java/weka.jar
# ~/jython2.5.3/jython run-flow.py
from __future__ import with_statement
import weka.gui.beans.FlowRunner as FlowRunner
import weka.core.Environment as Environment
import re, csv, sys, fileinput, os

fr = FlowRunner()
env = Environment()

path = os.getcwd() + '/'
cur_dir = os.path.dirname(os.path.realpath(__file__)) + '/'

env.addVariable('UnifiedFlow.InputCSV', path + sys.argv[1])
env.addVariable('UnifiedFlow.OutputCSV', path + sys.argv[2] + '-export.txt')

fr.loadXML(cur_dir + 'unified-model.kfml')
fr.setEnvironment(env)
fr.run()
fr.waitUntilFinished()

print('Flows Complete')

for line in fileinput.FileInput(path + sys.argv[2] + '-export.txt',inplace=1):
    line = line.replace("${UnifiedFlow.InputCSV}", os.path.splitext(sys.argv[1])[0])
    print line,

fp = open(path + sys.argv[2] + '-export.txt', 'r')
msgs = []
cur = ''

for line in fp:
    if line.strip('\r\n= ') == 'Evaluation result':
        msgs.append(cur)
        cur = ''
    
    cur += line
msgs.append(cur)
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

    if not r2_re:
        tup = (scheme_re.group(1), scheme_re.group(2), relation_re.group(1), rmse_re.group(1), correct_re.group(1), fscore_re.group(1))
        nom.append(tup)
    else:
        tup = (scheme_re.group(1), scheme_re.group(2), relation_re.group(1), rmse_re.group(1), '', '', r2_re.group(1))
        num.append(tup)

    
csvfp = open(sys.argv[2] + '-summary.csv', 'wb')
csvwriter = csv.writer(csvfp)

csvwriter.writerow(('Desc','Classifier','Relation','RMSE','Percent Correct','F-Score','Correlation coeff'))

nom.sort(key=lambda x: x[0])
num.sort(key=lambda x: x[0])

for n in nom:
    csvwriter.writerow(n)

for n in num:
    csvwriter.writerow(n)

csvfp.close()

sys.exit(0)