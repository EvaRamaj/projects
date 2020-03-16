"use strict";

import React from 'react';
import { Card, Button, TextField, Grid, Cell, Subheader, SelectField } from 'react-md';
import { withRouter, Link } from 'react-router-dom';

import Page from './Page';
import 'react-md/dist/react-md.indigo-pink.min.css'
import FormControl from "react-bootstrap/es/FormControl";
import CompanyService from '../services/CompanyService';
import CompanySearchResults from "./CompanySearchResults";
import UserService from "../services/UserService";

const style = { maxWidth: 500 };


class CompanySearch extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            searchterm: "",
            results: [],
            user: UserService.isAuthenticated() ? UserService.getCurrentUser() : undefined
        };

        this.handleChangeSearchTerm = this.handleChangeSearchTerm.bind(this);
        this.handleSearch = this.handleSearch.bind(this);

        //this.handleSearch("");
    }

    componentWillReceiveProps(nextProps)
    {
        this.setState({
            searchterm: nextProps.searchterm,
            results: nextProps.results
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
            this.setState(Object.assign({}, this.state, {results: data.hits.hits}));
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
                <h1 class="text-center" style={{paddingTop:"1em"}}>For Companies</h1>
                <h2 class="text-center">We help companies to enlarge their network</h2>
                <h2 className="text-center">by matching with the best Retired Professionals they need.</h2>
                <div className="col-md-12">
                    <div className="col-md-3"/>
                    <div className="col-md-6">
                        <form className="md-grid" style={{marginTop: 25 + 'px'}}>
                            <FormControl
                                id="search"
                                type="text"
                                value={this.state.searchterm}
                                placeholder="Sales, Automotive, Marketing"
                                onChange={this.handleChangeSearchTerm}
                            />
                            {/*<Button id="submit" type="button" raised primary className="md-cell--center" style={{marginTop: 50 + 'px'}}>See our pool of talents</Button>
                            <AlertMessage className="md-row md-full-width" >{this.props.error ? `${this.props.error}` : ''}</AlertMessage>*/}
                        </form>

                        {this.state.user && this.state.user.role==="company" ? [] : [
                            <div>
                                <h3 className="text-center" style={{color: 'grey', marginTop: 40 + 'px'}}>GET THE BEST MATCH</h3>
                                <div className="col-md-4"/>
                                <div className="col-md-4">
                                    <div className="col-md-2"/>
                                    <div>
                                        <Button raised secondary className="md-cell--middle" onClick={() => this.props.history.replace({pathname:'/company/login',state:{searchtext: this.state.searchterm}})}>Register your project</Button>
                                    </div>
                                </div><div className="col-md-4"/>
                            </div> ]}
                    </div>
                    <div className="col-md-3"/>
                </div>
                <div className="col-md-12" style={{margin: 25 + 'px'}}>
                    <CompanySearchResults results={this.state.results} user={this.state.user}/>
                </div>
            </Page>
        );
    }
};

export default withRouter(CompanySearch);
