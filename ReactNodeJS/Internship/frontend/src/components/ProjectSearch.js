"use strict";

import React from 'react';
import { withRouter, Link } from 'react-router-dom';

import Page from './Page';
import 'react-md/dist/react-md.indigo-pink.min.css'
import FormControl from "react-bootstrap/es/FormControl";
import ProjectService from '../services/ProjectService';
import UserService from "../services/UserService";
import ProjectSearchResults from "./ProjectSearchResults";

const style = { maxWidth: 500 };

class ProjectSearch extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            searchterm: "",
            results: [],
            projectCount: 0,
            userCount: 0,
            user: UserService.isAuthenticated() ? UserService.getCurrentUser() : undefined
        };

        this.handleChangeSearchTerm = this.handleChangeSearchTerm.bind(this);
        this.handleSearch = this.handleSearch.bind(this);
    }

    componentWillReceiveProps(nextProps)
    {
        this.setState({
            searchterm: nextProps.searchterm,
            results: nextProps.results,
            projectCount: nextProps.projectCount,
            userCount: nextProps.userCount
        })
    }

    //For React-bootstrap form controls
    handleChangeSearchTerm(e) {
        this.setState(Object.assign({}, this.state, {searchterm: e.target.value}));
        this.handleSearch(e.target.value);
    }

    handleSearch(value) {
        //event.preventDefault();
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
            this.setState(Object.assign({}, this.state, {results: data.hits.hits}));
            this.setState(Object.assign({}, this.state, {projectCount: data.hits.total}));
        }).catch((e) => {
            console.error(e);
            this.setState({
                error: e
            });
        });
    }

    render() {
        return (
            <Page>
                <div className="col-md-12" style ={{backgroundColor:"#E6F4F6"}}>
                    <div className="col-md-12">
                        <h1 className="text-center">Our latest projects</h1>
                        <h2 className="text-center">We connect retired professionals to interesting & tailored projects.</h2>
                    </div>
                    <div className="col-md-12">
                        <div className="col-md-3"/>
                        <div className="col-md-6">
                            <form className="md-grid" style={{marginTop: 50 + 'px',marginBottom: 50 + 'px'}}>
                                <FormControl
                                    id="search"
                                    type="text"
                                    value={this.state.searchterm}
                                    placeholder="Title, Company, Industry"
                                    onChange={this.handleChangeSearchTerm}
                                />
                            </form>
                        </div>
                        <div className="col-md-3"/>
                    </div>
                    <div className="col-md-12" style={{margin: 25 + 'px'}}>
                        <ProjectSearchResults results={this.state.results} user={this.state.user} projectCount={this.state.projectCount} userCount={this.state.userCount}/>
                    </div>
                </div>
            </Page>
        );
    }
};

export default withRouter(ProjectSearch);
