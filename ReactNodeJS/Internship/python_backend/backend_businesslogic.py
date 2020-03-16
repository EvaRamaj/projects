#import distance as d
import backend_data as db
from nltk.corpus import stopwords 
from nltk.tokenize import word_tokenize 
from nltk.tokenize import RegexpTokenizer
import string
import nltk
import spacy
import re
import json
from flask import jsonify
from google.cloud import language
from google.oauth2 import service_account
import argparse
import sys
from google.cloud.language import enums
from google.cloud.language import types
import six
from google.cloud import storage
import time as t


nltk.download('averaged_perceptron_tagger')
nltk.download('punkt')
nltk.download('stopwords')
stop_words = set(stopwords.words('english'))
spacy.cli.download('en_core_web_sm') 
spacy.prefer_gpu()
nlp = spacy.load('en_core_web_sm')

creds = service_account.Credentials.from_service_account_file('dps-batches-batch5-4fc777a5c6b8--wayer.json')
client = language.LanguageServiceClient(credentials=creds)

def preprocess(sentence):
    sentence = remove_special_characters(sentence)
    word_tokens = word_tokenize(sentence)
    filtered_words = [w for w in word_tokens if not w in stopwords.words('english')]
    
    return " ".join(filtered_words)

def remove_special_characters(s):
    tokenizer = nltk.RegexpTokenizer('[a-zA-Z]\w+\'?\w*')
    tokens= tokenizer.tokenize(s)
    return " ".join(tokens)

def entities_text(text):
    """Detects entities in the text."""

    if isinstance(text, six.binary_type):
        text = text.decode('utf-8')

    # Instantiates a plain text document.
    document = types.Document(
        content=text,
        type=enums.Document.Type.PLAIN_TEXT)

    # Detects entities in the document. You can also analyze HTML with:
    #   document.type == enums.Document.Type.HTML
    entities = client.analyze_entities(document).entities

    # entity types from enums.Entity.Type
    entity_type = ('UNKNOWN', 'PERSON', 'LOCATION', 'ORGANIZATION',
                   'EVENT', 'WORK_OF_ART', 'CONSUMER_GOOD', 'OTHER')

    my_entity_type=["UNKNOWN","OTHER"]
    my_entity_list=[]
    my_keyword_list=[]
    
    for entity in entities:
        if entity_type[entity.type] not in my_entity_type:
            continue
        my_entity_list.append((entity.name,round(entity.salience,2)))
    my_entity_list.sort(key=lambda tup: tup[1])
    my_keyword_list=[s[0] for n,s in enumerate(my_entity_list)]
    
    #print(my_entity_list)
    return " ".join(my_keyword_list)


def remove_dublicates_words(sent):
  word_tokens = word_tokenize(sent)
  my_list=[]
  for w in word_tokens:
    if w not in my_list:
      my_list.append(w)
      
  return " ".join(my_list)


def get_keywords(sentence):
    key_words2=syntax_text(sentence)[1]['keywords']
    return " ".join(key_words2) 


def syntax_text(text):
    my_token_list=[]
    my_key_words=[]
    alphabet=list(string.ascii_lowercase)
    
    my_token_dict={}
    my_keywords_dict={}
    
    if isinstance(text, six.binary_type):
        text = text.decode('utf-8')

    # Instantiates a plain text document.
    document = types.Document(
        content=text,
        type=enums.Document.Type.PLAIN_TEXT)

    # Detects syntax in the document. You can also analyze HTML with:
    #   document.type == enums.Document.Type.HTML
    tokens = client.analyze_syntax(document).tokens

    # part-of-speech tags from enums.PartOfSpeech.Tag
    pos_tag = ('UNKNOWN', 'ADJ', 'ADP', 'ADV', 'CONJ', 'DET', 'NOUN', 'NUM',
               'PRON', 'PRT', 'PUNCT', 'VERB', 'X', 'AFFIX')
    for token in tokens:

        if token.text.content.strip() in alphabet:
            continue

        if token.text.content in my_token_list:
            continue

        my_token_list.append((pos_tag[token.part_of_speech.tag],
                               token.text.content))
        
        if any(str.isdigit(c) for c in token.text.content):
            continue

        if token.text.content in my_key_words:
            continue

        if pos_tag[token.part_of_speech.tag]=='NOUN':
            my_key_words.append(token.text.content)
            
    my_token_dict['Token_Tags']= my_token_list
    my_keywords_dict['keywords']=my_key_words
    
    return my_token_dict,my_keywords_dict

def similarity_score_with_spacy(sent1,sent2):
    sent1_tokens = nlp(u'{}'.format(sent1))
    sent2_tokens = nlp(u'{}'.format(sent2))

    count=0
    sim_tot=0
    for token1 in sent1_tokens:
        for token2 in sent2_tokens:
            count=+1
            sim_tot=+ float(token1.similarity(token2))

    if(count==0):
        return 0

    return round((sim_tot/count),2)

def check_similarity(personal_profile,job_details):
        scores=[]

        #score without keywords
        job_processed=preprocess(job_details)
        profile_processed=preprocess(personal_profile)

        #score with keywors
        job_keywords=get_keywords(job_details)
        profile_keywords=get_keywords(personal_profile)

        #scores
        normal_score= similarity_score_with_spacy(job_processed,profile_processed)
        scores.append(('normal_score',normal_score))
        keywords_based_score=similarity_score_with_spacy(job_keywords,profile_keywords)
        scores.append(('keyword_based_score',keywords_based_score))
        
        return scores


def get_key_words_from_project(project_details):
   
    overview=project_details['overview']
    responsibilities=project_details['responsibilities']
    merged_text=responsibilities+' '+overview
    
    merged_text=entities_text(merged_text)
    #word_tokens = word_tokenize(merged_text)
    key_words=get_keywords(merged_text)
    
    return key_words

def get_key_words_from_project_as_token(project_details):
    return word_tokenize(project_details)

def get_key_words_from_profile_as_token(profile_details):
    return word_tokenize(profile_details)


def get_key_words_from_profile(profile_details):
    
    #profile_details=db.get_all_profiles()
    profile_details_without_key=profile_details#list(profile_details.values())[0]
    interests=profile_details_without_key[0]
    description=profile_details_without_key[1]
    
    merged_text=str(interests)+' '+str(description)

    #Trying to remove unnecessary words
    merged_text=merged_text.replace("compName"," ")
    merged_text=merged_text.replace("description"," ")
    merged_text=merged_text.replace("projects"," ")
    merged_text=merged_text.replace("title"," ")
    merged_text=merged_text.replace("Z"," ")

    merged_text=entities_text(merged_text)
    key_words=get_keywords(merged_text)
    return key_words


def top_profiles(project_details,list_profiles):
    profile_scores_list=[]
    profile_scores_dic={}
    dict_keywords={}

    for p in list_profiles:
        user_dic={}
        k=list(p.keys())[0]
        v=list(p.values())[0]

        st=t.time()
        score=check_similarity(get_key_words_from_project(project_details),get_key_words_from_profile(v))
        print("Similarity check Time: ",(t.time()-st))

        user_dic["id"]=k
        user_dic["normal_score"]=score[0][1]
        user_dic["keyword_based_score"]=score[1][1]
        
        #for testing only
        user_dic["user_keyword"]=word_tokenize(get_key_words_from_profile(v))

        profile_scores_list.append(user_dic)
    
    key_words=get_key_words_from_project_as_token(get_key_words_from_project(project_details))
    profile_scores_list=sorting_list_of_dictionaries(profile_scores_list)
    profile_scores_dic["results"]=profile_scores_list[:3]
    dict_keywords["keywords"]=remove_dublicates(key_words)

    return profile_scores_dic,dict_keywords

def sorting_list_of_dictionaries(mylist):
    user_dic={}
    profile_scores_dict=[]
    profile_scores=[]
    
    for i,p in enumerate(mylist):
        mean=round((p.get('normal_score')+p.get('keyword_based_score'))/2,2)
        profile_data=(p.get('id'),p.get('normal_score'),p.get('keyword_based_score'),mean,p.get('user_keyword'))
        profile_scores.append(profile_data)
    
    
    profile_scores.sort(key=lambda tup: tup[3],reverse=True)
    
    for arr in profile_scores:
        user_dic["id"]=arr[0]
        user_dic["normal_score"]=arr[1]
        user_dic["keyword_based_score"]=arr[2]
        #user_dic["keyword"]=arr[4]
        profile_scores_dict.append(user_dic)
        user_dic={}
       
    
    return profile_scores_dict

def remove_dublicates(bag_of_words):
  my_list=[]
  for w in bag_of_words:
    if w not in my_list:
      my_list.append(w)
      
  return my_list


def main(project_id):
    try:
        st=t.time()
        personal_profiles=db.get_all_profiles()
        project_details=db.get_project_details(project_id)
        print("reading data time: ",(t.time()-st))
        st=t.time()
        _top_profiles=top_profiles(project_details,personal_profiles)
        print("processing profiles Time: ",(t.time()-st))

        if(len(_top_profiles)<1):
            return []

        return _top_profiles
    except Exception as e:
        print("Oops!  Invalid project id.  Try again...")
        return []
        

