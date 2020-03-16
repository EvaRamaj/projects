from datetime import datetime
from elasticsearch import Elasticsearch
from pprint import pprint
import config



try:
    #connection to elastic search
    es = Elasticsearch(
        [config.ES['host']],
        http_auth=(config.ES['elasticSearchUsername'], config.ES['elasticSearchPassword']),
        scheme=config.ES['scheme'],
        port=config.ES['port'],
    )
    print('connected')
except Exception as e:
    print('not connetected. ',e)


def get_project_details(id):
    search_result= es.search(index='project',body={"query":{
                    "match":{
                        "_id":id
                        }
                    }
                })['hits']['hits'][0]['_source']
    
    return search_result

def get_all_profiles():
    search_result= es.search(index='user',body={"query":{
                    "match_all":{
                        }
                    }
                })['hits']['hits']
    
    return  preprocessed_profile_data(search_result)

def preprocessed_profile_data(profiles):
    my_data_list=[]
    for u in profiles:
        temp_data_list=[]
        temp_profile_dict={}
        temp_data_list.append(u['_source']['profileData']['interest'])
        temp_data_list.append(u['_source']['profileData']['experience'])
        temp_profile_dict[u['_id']]=temp_data_list

        my_data_list.append(temp_profile_dict)

    return my_data_list
