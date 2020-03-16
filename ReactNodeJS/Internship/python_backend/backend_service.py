from flask import Flask, redirect, url_for, request
from flask import jsonify
import config
import backend_businesslogic as bl

app = Flask(__name__)
app.config.update(SERVER_NAME='0.0.0.0:{}'.format(config.App['port']))

@app.route('/profiles',methods = ['GET'])
def get_profiles():
    recommended_profiles={}

    #Geting the profileId from the query string
    projectid = request.args.get('projectid')
     #Adding the profileId and Score to the dictionary 
    recommended_profiles=bl.main(projectid)


    return jsonify(recommended_profiles)

if __name__ == '__main__':
   app.run(debug = True)
