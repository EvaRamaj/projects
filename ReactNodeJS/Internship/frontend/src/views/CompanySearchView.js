"use strict";

import React from 'react';

import CompanySearch from '../components/CompanySearch';

import CompanyService from '../services/CompanyService';


export class CompanySearchView extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            searchterm: "",
            results: []
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
                        { "match_phrase_prefix" : { "profileData.experience.description" : value } },
                        { "match_phrase_prefix" : { "profileData.experience.compName" : value } },
                        { "match_phrase_prefix" : { "profileData.interest.industries" : value } },
                        { "match_phrase_prefix" : { "profileData.interest.projects" : value } }
                    ]
                }
            }
        };

        if (value.length===0)
            query.query = { "match_all" : {}};

        CompanyService.search(query).then((data) => {
            this.setState({
                searchterm: value,
                results: data.hits.hits
            });
        }).catch((e) => {
            this.setState({
                error: e
            });
        });
    }

    render() {
        return (
            <CompanySearch searchterm={this.state.searchterm} results={this.state.results} error={this.state.error}/>
        );
    }
}
