"use strict";

import React from 'react';
import CompanyService from "../services/CompanyService";
import MatchingResults from '../components/MatchingResults';
import ProjectService from "../services/ProjectService";


export class MatchingResultsView extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            users:this.props.history.location.state.users,
            results: [],
            project:[]
        }
    }

    componentWillMount(props)
    {
        let id = this.props.match.params.id;

        let value = [];
        let i;
        for (i=0;i<this.state.users.length;i++)
            value.push(this.state.users[i].id);
        let query = {
            "query" : {
                "terms" : {"_id": value }
                    }
        };

        CompanyService.search(query).then((data) => {
            this.setState({
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
            <MatchingResults results={this.state.results} users={this.state.users} error={this.state.error} />
        );
    }
}