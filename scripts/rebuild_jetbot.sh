FILE=$1
INIT='__init__.py'
JBR='/opt/jetbot/jetbot/'
cp ${FILE} ${JBR}
cp ${INIT} ${JBR}

cd /opt/jetbot && python3 setup.py install
cd /workspace/jetbot
