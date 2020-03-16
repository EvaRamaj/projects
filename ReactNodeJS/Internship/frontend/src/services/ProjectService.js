"use strict";

import HttpService from "./HttpService";

export default class ProjectService {

    constructor() {
    }

    static baseURL() {return HttpService.apiURL()+"/project"; }

    static search(query){
        return new Promise((resolve, reject) => {
            HttpService.post(`${ProjectService.baseURL()}/search`, query , function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static getProject(id) {
        return new Promise((resolve, reject) => {
            HttpService.get(`${ProjectService.baseURL()}/${id}`, function(data) {
                if(data != undefined || Object.keys(data).length !== 0) {
                    resolve(data);
                }
                else {
                    reject('malakia_1 : We cannot get the project');
                }
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }

    static create(project) {
        return new Promise((resolve, reject) => {
            HttpService.post(`${ProjectService.baseURL()}/create`, project, function(data) {
                resolve(data);
            }, function(textStatus) {
                reject(textStatus);
            });
        });
    }
}
