"use strict";

import React from 'react';

import ProjectSearch from '../components/ProjectSearch';

import ProjectService from '../services/ProjectService';
import UserService from '../services/UserService';

export class ProjectSearchView extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            searchterm: "",
            results: [],
            projectCount: 0,
            userCount: 0
        };
    }

    componentWillMount(props)
    {
        let value="";
        if (this.props.history.location.state!==undefined)
        {
            value=this.props.history.location.state.searchtext;
        }

        let query = {
            "query" : {
                "bool" : {
                    "should": [
                        { "match_phrase_prefix" : { "title" : value } },
                        { "match_phrase_prefix" : { "company.name" : value } }
                    ]
                }
            }
        };

        if (value.length===0)
            query.query = { "match_all" : {}};

        ProjectService.search(query).then((data) => {
            this.setState({
                searchterm: value,
                results: data.hits.hits,
                projectCount: data.hits.total
            }, () => {
                UserService.getUserCount().then((totalUsers) => {
                    this.setState({
                        userCount: totalUsers.count
                    })
                }).catch((e) => {
                    this.setState({
                        error: e
                    });
                });
            });
        }).catch((e) => {
            this.setState({
                error: e
            });
        });
    }

    render() {
        return (
            <ProjectSearch searchterm={this.state.searchterm} results={this.state.results} projectCount={this.state.projectCount} userCount={this.state.userCount} error={this.state.error}/>
        );
    }
}
